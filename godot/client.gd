extends Node3D
## BeaconSim 앱 (Godot 프런트엔드, 1단계). 백엔드(beacon_sim/server.py)와 TCP 로 연결.
## 화면: 3D 월드(고품질) + 센서 표시 + 재생 바 / 상황판 / 이벤트 / 카메라 패널. 조작: 마우스 시점, 방향키 비콘 주행.

const BeaconWorld = preload("res://world.gd")

# 색 (ui_style.py 와 같은 팔레트)
const BG0 := Color(0.055, 0.067, 0.086, 0.92)
const BG1 := Color(0.082, 0.098, 0.125, 1.0)
const BG2 := Color(0.118, 0.137, 0.170, 1.0)
const BG3 := Color(0.160, 0.185, 0.225, 1.0)
const TEXT := Color(0.902, 0.918, 0.941)
const MUTED := Color(0.545, 0.588, 0.647)
const ACCENT := Color(0.302, 0.639, 1.0)
const TEAL := Color(0.169, 0.831, 0.773)
const OK := Color(0.239, 0.863, 0.518)
const WARN := Color(1.0, 0.722, 0.302)
const DANGER := Color(1.0, 0.353, 0.373)
const SCC := Color(0.780, 0.490, 1.0)
const AMBER := Color(1.0, 0.600, 0.200)
# Font Awesome 6 Solid
const I_PLAY := 0xf04b
const I_PAUSE := 0xf04c
const I_ROTATE := 0xf01e
const I_MAP := 0xf279
const I_CLOCK := 0xf017
const I_LINK := 0xf0c1
const I_DOT := 0xf192
const I_BULLSEYE := 0xf140
const I_WIFI := 0xf1eb
const I_SAT := 0xf7bf
const I_CAMERA := 0xf030
const I_GAMEPAD := 0xf11b
const I_TOWER := 0xf519
const I_CAR := 0xf5e4
const I_BURST := 0xf5e1
const I_INFO := 0xf05a
const I_XMARK := 0xf00d

var args := {}
var tcp := StreamPeerTCP.new()
var rx := PackedByteArray()
var world: Node3D = null
var world_seq := -1
var world_msg := {}
var hello := {}
var st := {}                       # 최근 state
var main_cam: Camera3D
var neutral_env: Environment
var sensor_sv: SubViewport
var sensor_cam: Camera3D
var rain_main: GPUParticles3D
var rain_sensor: GPUParticles3D
var orbit := {"target": Vector3.ZERO, "dist": 30.0, "yaw": 0.0, "pitch": deg_to_rad(20)}
var follow := true
var show := {"pandar": true, "tele": true, "radar": true, "gnss": true, "camera": true, "scc": true,
	"labels": true, "lanelet": false, "env_view": true, "auto_roof": true}
var veh_dims: Array = []
var robot_T := Transform3D()
var drive_t := 0.0
var cam_next := 0.0
var cam_busy := false
# 오버레이
var pc_nodes := {}
var radar_pts: MeshInstance3D
var radar_boxes: MultiMeshInstance3D
var radar_edges: MultiMeshInstance3D
var gnss_mesh: MeshInstance3D
var gnss_trail := PackedVector3Array()
var gnss_trail_col := PackedColorArray()
var truth_trail := PackedVector3Array()
var trail_mesh: MeshInstance3D
var last_gnss_t := -1.0
var scc_mesh: MeshInstance3D
var lane_mesh: MeshInstance3D
var acc_mesh: MeshInstance3D
var lbl_acc: Label3D
var lbl_beacon: Label3D
# UI
var font_body: FontFile
var font_bold: FontFile
var ui: Control
var loading: Label
var btn_pause: Button
var speed_btns: Array = []
var lbl_time: Label
var lbl_env: Label
var lbl_cmd: Label
var lbl_ros: Label
var toggles := {}
var dash := {}
var log_box: VBoxContainer
var log_lines: Array = []
var cam_panel: PanelContainer
var shot_path := ""
var shot_at := -1.0

# ================================================================== 시작
func start(a: Dictionary) -> void:
	args = a
	shot_path = args.get("shot", "")
	DisplayServer.window_set_title("BeaconSim")
	get_window().min_size = Vector2i(1100, 700)
	main_cam = Camera3D.new()
	main_cam.fov = 50.0
	main_cam.near = 0.3
	main_cam.far = 9000.0
	main_cam.current = true
	add_child(main_cam)
	neutral_env = _neutral_env()
	_build_overlays()
	_build_ui()
	get_window().size_changed.connect(_on_resize)
	_on_resize()
	tcp.connect_to_host("127.0.0.1", int(args.get("port", "0")))

func _notification(what: int) -> void:
	if what == NOTIFICATION_WM_CLOSE_REQUEST:
		_send({"type": "quit"})
		get_tree().quit()

func _on_resize() -> void:
	var s := get_window().size
	var z := clampf(minf(s.y / 1000.0, s.x / 1700.0), 0.75, 3.0)
	get_window().content_scale_factor = snappedf(z, 0.05)

func _neutral_env() -> Environment:
	var e := Environment.new()
	var sky := Sky.new()
	var sm := ProceduralSkyMaterial.new()
	sky.sky_material = sm
	e.background_mode = Environment.BG_SKY
	e.sky = sky
	e.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	e.tonemap_mode = Environment.TONE_MAPPER_ACES
	e.ssao_enabled = true
	return e

# ================================================================== 네트워크
func _send(d: Dictionary) -> void:
	if tcp.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		return
	var js := JSON.stringify(d).to_utf8_buffer()
	var h := PackedByteArray()
	h.resize(8)
	h.encode_u32(0, js.size())
	h.encode_u32(4, 0)
	tcp.put_data(h + js)

func _net() -> void:
	tcp.poll()
	var s := tcp.get_status()
	if s == StreamPeerTCP.STATUS_ERROR or (s == StreamPeerTCP.STATUS_NONE and world_seq >= 0):
		get_tree().quit()
		return
	if s != StreamPeerTCP.STATUS_CONNECTED:
		return
	var n := tcp.get_available_bytes()
	if n > 0:
		var r := tcp.get_data(n)
		rx.append_array(r[1])
	var off := 0
	var latest_state := {}
	var latest_bin := PackedByteArray()
	while rx.size() - off >= 8:
		var jl := rx.decode_u32(off)
		var bl := rx.decode_u32(off + 4)
		if rx.size() - off < 8 + jl + bl:
			break
		var msg = JSON.parse_string(rx.slice(off + 8, off + 8 + jl).get_string_from_utf8())
		var bin := rx.slice(off + 8 + jl, off + 8 + jl + bl)
		off += 8 + jl + bl
		if typeof(msg) != TYPE_DICTIONARY:
			continue
		if msg["type"] == "state":          # 상태는 마지막 것만 적용
			latest_state = msg
			latest_bin = bin
		else:
			_handle(msg, bin)
	if off > 0:
		rx = rx.slice(off)
	if not latest_state.is_empty():
		_on_state(latest_state, latest_bin)

func _arr(msg: Dictionary, bin: PackedByteArray, name: String) -> PackedByteArray:
	var d: Array = msg["arrays"][name]
	return bin.slice(int(d[0]), int(d[0]) + int(d[1]))

func _handle(m: Dictionary, bin: PackedByteArray) -> void:
	match m["type"]:
		"hello":
			hello = m
			_load_fonts(m["fonts"], m["icons"])
			_setup_sensor_camera(m["camera"])
		"world":
			_on_world(m, bin)
		"pc":
			_on_pc(m, bin)
		"radar":
			_on_radar(m, bin)
		"log":
			for ln in m["lines"]:
				_add_log(ln)
		"cam_req":
			_capture_for_ros(m)
		"bye":
			get_tree().quit()

# ================================================================== 월드
func _on_world(m: Dictionary, bin: PackedByteArray) -> void:
	loading.visible = true
	loading.text = "장면 불러오는 중..."
	await get_tree().process_frame
	if world != null:
		world.queue_free()
	world = BeaconWorld.new()
	add_child(world)
	world.build(m["scene"])
	world_seq = int(m["seq"])
	world_msg = m
	veh_dims = m["veh_dims"]
	rain_main = world.make_weather()
	rain_sensor = world.make_weather()
	_set_main_env()
	for k in pc_nodes:
		pc_nodes[k].mesh = null
	gnss_trail = PackedVector3Array()
	gnss_trail_col = PackedColorArray()
	truth_trail = PackedVector3Array()
	last_gnss_t = -1.0
	if m.has("arrays") and m["arrays"].has("lane_pts"):
		var p := _arr(m, bin, "lane_pts").to_vector3_array()
		var c := _arr(m, bin, "lane_cols").to_color_array()
		lane_mesh.mesh = _prim_mesh(Mesh.PRIMITIVE_LINES, p, c)
	var a: Array = m["accident"]
	var ap := Vector3(a[0], a[1], a[2])
	acc_mesh.mesh = _prim_mesh(Mesh.PRIMITIVE_LINES, PackedVector3Array([ap, ap + Vector3(0, 6, 0)]),
		PackedColorArray([AMBER, AMBER]))
	lbl_acc.position = ap + Vector3(0, 6.6, 0)
	loading.visible = false
	follow = true
	_preset("beacon")

func _set_main_env() -> void:
	main_cam.environment = null if show["env_view"] else neutral_env

# ================================================================== 상태
func _on_state(m: Dictionary, bin: PackedByteArray) -> void:
	if int(m["world"]) != world_seq or world == null:
		return
	st = m
	robot_T = BeaconWorld.xf(m["robot"])
	world.set_robot(m["robot"])
	world.set_vehicles(_arr(m, bin, "veh_idx").to_int32_array(), _arr(m, bin, "veh_T").to_float32_array())
	if sensor_cam:
		sensor_cam.transform = BeaconWorld.xf(m["cam"])
	_update_scc(m, bin)
	_update_gnss(m.get("gnss"))
	lbl_beacon.position = robot_T.origin + Vector3(0, 1.8, 0)
	if follow:
		orbit["target"] = robot_T.origin + Vector3(0, 0.5, 0)
	_update_ui()

# ================================================================== 오버레이
func _pt_mat(size: float, alpha := false) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.vertex_color_use_as_albedo = true
	m.use_point_size = true
	m.point_size = size
	m.disable_fog = true
	if alpha:
		m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	return m

func _prim_mesh(prim: int, p: PackedVector3Array, c: PackedColorArray) -> ArrayMesh:
	var mesh := ArrayMesh.new()
	if p.size() == 0:
		return mesh
	var arr := []
	arr.resize(Mesh.ARRAY_MAX)
	arr[Mesh.ARRAY_VERTEX] = p
	arr[Mesh.ARRAY_COLOR] = c
	mesh.add_surface_from_arrays(prim, arr)
	return mesh

func _mi(mat: Material) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	mi.material_override = mat
	mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	mi.layers = 4                         # 레이어 3: 센서 카메라에는 안 보임
	add_child(mi)
	return mi

func _build_overlays() -> void:
	for k in ["pandar", "tele"]:
		pc_nodes[k] = _mi(_pt_mat(2.0))
	radar_pts = _mi(_pt_mat(7.0))
	var bm := BoxMesh.new()
	bm.size = Vector3.ONE
	var fill := StandardMaterial3D.new()
	fill.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	fill.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	fill.albedo_color = Color(1.0, 0.45, 0.0, 0.45)
	fill.disable_fog = true
	fill.cull_mode = BaseMaterial3D.CULL_DISABLED
	bm.material = fill
	radar_boxes = MultiMeshInstance3D.new()
	radar_boxes.layers = 4
	radar_boxes.multimesh = MultiMesh.new()
	radar_boxes.multimesh.transform_format = MultiMesh.TRANSFORM_3D
	radar_boxes.multimesh.mesh = bm
	add_child(radar_boxes)
	# 상자 모서리 (단위 정육면체 선)
	var ep := PackedVector3Array()
	var cs := [Vector3(-.5, -.5, -.5), Vector3(.5, -.5, -.5), Vector3(.5, .5, -.5), Vector3(-.5, .5, -.5),
		Vector3(-.5, -.5, .5), Vector3(.5, -.5, .5), Vector3(.5, .5, .5), Vector3(-.5, .5, .5)]
	for e in [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]]:
		ep.append(cs[e[0]])
		ep.append(cs[e[1]])
	var ec := PackedColorArray()
	ec.resize(ep.size())
	ec.fill(Color(1.0, 0.55, 0.1))
	var em := _prim_mesh(Mesh.PRIMITIVE_LINES, ep, ec)
	em.surface_set_material(0, _pt_mat(1.0))
	radar_edges = MultiMeshInstance3D.new()
	radar_edges.layers = 4
	radar_edges.multimesh = MultiMesh.new()
	radar_edges.multimesh.transform_format = MultiMesh.TRANSFORM_3D
	radar_edges.multimesh.mesh = em
	add_child(radar_edges)
	gnss_mesh = _mi(_pt_mat(1.0))
	trail_mesh = _mi(_pt_mat(3.0))
	scc_mesh = _mi(_pt_mat(9.0))
	lane_mesh = _mi(_pt_mat(1.0))
	acc_mesh = _mi(_pt_mat(1.0))
	lbl_acc = _label3d("사고 현장", AMBER)
	lbl_beacon = _label3d("비콘", WARN)

func _label3d(text: String, col: Color) -> Label3D:
	var l := Label3D.new()
	l.text = "●  " + text
	l.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	l.no_depth_test = true
	l.fixed_size = true
	l.pixel_size = 0.0006
	l.font_size = 24
	l.outline_size = 10
	l.modulate = col
	l.outline_modulate = Color(0.05, 0.06, 0.08, 0.85)
	l.layers = 4
	l.set_meta("col", col)
	add_child(l)
	return l

func _on_pc(m: Dictionary, bin: PackedByteArray) -> void:
	var name: String = m["name"]
	if not pc_nodes.has(name):
		return
	var xyz := _arr(m, bin, "xyz").to_vector3_array()
	var col := _arr(m, bin, "col").to_color_array()
	pc_nodes[name].mesh = _prim_mesh(Mesh.PRIMITIVE_POINTS, xyz, col)

func _on_radar(m: Dictionary, bin: PackedByteArray) -> void:
	var xyz := _arr(m, bin, "xyz").to_vector3_array()
	var c := PackedColorArray()
	c.resize(xyz.size())
	c.fill(Color(1.0, 0.15, 0.15))
	radar_pts.mesh = _prim_mesh(Mesh.PRIMITIVE_POINTS, xyz, c)
	var b := _arr(m, bin, "boxes").to_float32_array()
	var n := b.size() / 12
	for mm in [radar_boxes.multimesh, radar_edges.multimesh]:
		mm.instance_count = n
		for k in range(n):
			mm.set_instance_transform(k, BeaconWorld.xf_at(b, k))

func _update_gnss(g) -> void:
	if g == null or typeof(g) != TYPE_DICTIONARY:
		gnss_mesh.mesh = null
		return
	var col := Color(g["color"][0], g["color"][1], g["color"][2])
	var p := Vector3(g["xyz"][0], g["xyz"][1], g["xyz"][2])
	if float(g["t"]) != last_gnss_t:
		last_gnss_t = float(g["t"])
		if g["valid"]:
			gnss_trail.append(p + Vector3(0, 0.1, 0))
			gnss_trail_col.append(col)
		truth_trail.append(Vector3(g["truth"][0], g["truth"][1] + 0.1, g["truth"][2]))
		if gnss_trail.size() > 600:
			gnss_trail = gnss_trail.slice(gnss_trail.size() - 600)
			gnss_trail_col = gnss_trail_col.slice(gnss_trail_col.size() - 600)
		if truth_trail.size() > 600:
			truth_trail = truth_trail.slice(truth_trail.size() - 600)
		var tp := truth_trail + gnss_trail
		var tc := PackedColorArray()
		tc.resize(truth_trail.size())
		tc.fill(Color(1, 1, 1, 0.6))
		trail_mesh.mesh = _prim_mesh(Mesh.PRIMITIVE_POINTS, tp, tc + gnss_trail_col)
	if not g["valid"]:
		gnss_mesh.mesh = null
		return
	var r := maxf(float(g["hacc"]) * 3.0, 0.3)
	var pts := PackedVector3Array()
	for k in range(32):
		var a0 := TAU * k / 32.0
		var a1 := TAU * (k + 1) / 32.0
		pts.append(p + Vector3(r * cos(a0), 0.15, -r * sin(a0)))
		pts.append(p + Vector3(r * cos(a1), 0.15, -r * sin(a1)))
	pts.append(p)
	pts.append(p + Vector3(0, 3, 0))
	var cs := PackedColorArray()
	cs.resize(pts.size())
	cs.fill(col)
	gnss_mesh.mesh = _prim_mesh(Mesh.PRIMITIVE_LINES, pts, cs)

func _update_scc(m: Dictionary, bin: PackedByteArray) -> void:
	if not show["scc"]:
		scc_mesh.mesh = null
		return
	var idx := _arr(m, bin, "veh_idx").to_int32_array()
	var T := _arr(m, bin, "veh_T").to_float32_array()
	var fl := _arr(m, bin, "veh_flags")
	var pts := PackedVector3Array()
	var cols := PackedColorArray()
	var lines := PackedVector3Array()
	var bpos := robot_T.origin + Vector3(0, 0.5, 0)
	for k in range(idx.size()):
		var f := fl[k]
		if not (f & 16):
			continue
		var col: Color
		if f & 1:
			col = DANGER
		elif f & 2:
			col = SCC
		elif f & 4:
			col = WARN
		elif f & 8:
			col = TEAL
		else:
			continue
		var tr := BeaconWorld.xf_at(T, k)
		var d: Array = veh_dims[idx[k]]
		pts.append(tr.origin + tr.basis.y * (float(d[2]) + 0.6))
		cols.append(col)
		if f & 2:
			lines.append(tr.origin + tr.basis.x * (float(d[0]) / 2.0) + tr.basis.y * 0.8)
			lines.append(bpos)
	var mesh := _prim_mesh(Mesh.PRIMITIVE_POINTS, pts, cols)
	if lines.size():
		var lc := PackedColorArray()
		lc.resize(lines.size())
		lc.fill(SCC)
		var arr := []
		arr.resize(Mesh.ARRAY_MAX)
		arr[Mesh.ARRAY_VERTEX] = lines
		arr[Mesh.ARRAY_COLOR] = lc
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, arr)
	scc_mesh.mesh = mesh

# ================================================================== 카메라 (센서)
func _setup_sensor_camera(c: Dictionary) -> void:
	sensor_sv = SubViewport.new()
	sensor_sv.size = Vector2i(int(c["wr"]), int(c["hr"]))
	sensor_sv.msaa_3d = Viewport.MSAA_4X
	sensor_sv.render_target_update_mode = SubViewport.UPDATE_DISABLED
	add_child(sensor_sv)
	sensor_cam = Camera3D.new()
	sensor_cam.keep_aspect = Camera3D.KEEP_HEIGHT
	sensor_cam.fov = float(c["vfov"])
	sensor_cam.near = 0.1
	sensor_cam.far = 3000.0
	sensor_cam.cull_mask = 1 | 2           # 오버레이(레이어 3) 제외, 터널(레이어 2)은 항상
	sensor_cam.current = true
	sensor_sv.add_child(sensor_cam)
	var tr: TextureRect = cam_panel.get_node("v/img")
	tr.texture = sensor_sv.get_texture()
	cam_panel.get_node("v/h/info").text = "%d×%d · %.0f Hz · HFOV %.0f°" % [int(c["width"]), int(c["height"]),
		float(c["rate_hz"]), float(c["hfov"])]

func _capture_for_ros(m: Dictionary) -> void:
	if sensor_sv == null or cam_busy:
		return
	cam_busy = true
	BeaconWorld.follow_weather(rain_sensor, sensor_cam)
	sensor_sv.render_target_update_mode = SubViewport.UPDATE_ONCE
	await RenderingServer.frame_post_draw
	var img := sensor_sv.get_texture().get_image()
	img.convert(Image.FORMAT_RGB8)
	var f := FileAccess.open(m["path"], FileAccess.WRITE)
	if f:
		f.store_buffer(img.get_data())
		f.close()
		_send({"type": "cam_done", "seq": m["seq"], "w": img.get_width(), "h": img.get_height()})
	cam_busy = false

# ================================================================== 시점 / 입력
func _preset(name: String) -> void:
	if name == "beacon":
		follow = true
		var fwd := robot_T.basis.x
		orbit["target"] = robot_T.origin + Vector3(0, 0.5, 0)
		orbit["dist"] = 22.0
		orbit["pitch"] = deg_to_rad(18)
		orbit["yaw"] = atan2(fwd.z, -fwd.x) + 0.35
	elif name == "overview" and world_msg.has("bounds"):
		follow = false
		var b: Array = world_msg["bounds"]
		var lo := Vector3(b[0][0], b[0][1], b[0][2])
		var hi := Vector3(b[1][0], b[1][1], b[1][2])
		orbit["target"] = (lo + hi) / 2.0
		orbit["dist"] = maxf(hi.x - lo.x, hi.z - lo.z) * 0.9
		orbit["pitch"] = deg_to_rad(70)
		orbit["yaw"] = deg_to_rad(-90)
	elif name == "tunnel" and world_msg.get("tunnels", []).size() > 0:
		follow = false
		var t: Array = world_msg["tunnels"][0]
		orbit["target"] = Vector3(t[0], t[1], t[2])
		orbit["dist"] = 70.0
		orbit["pitch"] = deg_to_rad(20)

func _unhandled_input(ev: InputEvent) -> void:
	if ev is InputEventMouseMotion:
		var mm := ev as InputEventMouseMotion
		var d := mm.relative
		if mm.button_mask & MOUSE_BUTTON_MASK_LEFT:
			orbit["yaw"] -= d.x * 0.005
			orbit["pitch"] = clampf(orbit["pitch"] + d.y * 0.005, deg_to_rad(2), deg_to_rad(89))
		elif mm.button_mask & (MOUSE_BUTTON_MASK_RIGHT | MOUSE_BUTTON_MASK_MIDDLE):
			follow = false
			var k: float = orbit["dist"] * 0.0015
			var y: float = orbit["yaw"]
			var fwd := Vector3(cos(y), 0, -sin(y))
			var right := Vector3(sin(y), 0, cos(y))
			orbit["target"] += (right * d.x - fwd * d.y) * k
	elif ev is InputEventMouseButton and ev.pressed:
		var mb := ev as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_WHEEL_UP:
			orbit["dist"] = clampf(orbit["dist"] * 0.88, 2.0, 8000.0)
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			orbit["dist"] = clampf(orbit["dist"] / 0.88, 2.0, 8000.0)
	elif ev is InputEventKey and ev.pressed and not ev.echo:
		match (ev as InputEventKey).physical_keycode:
			KEY_SPACE:
				_toggle_pause()
			KEY_R:
				_send({"type": "restart"})
			KEY_G:
				_send({"type": "restart", "map": true})
			KEY_F:
				follow = not follow
			KEY_1:
				_preset("beacon")
			KEY_2:
				_preset("overview")
			KEY_3:
				_preset("tunnel")

func _drive(dt: float) -> void:
	if get_viewport().gui_get_focus_owner() != null:
		return
	var v := 0.0
	var w := 0.0
	if Input.is_physical_key_pressed(KEY_UP):
		v += 1.5
	if Input.is_physical_key_pressed(KEY_DOWN):
		v -= 1.5
	if Input.is_physical_key_pressed(KEY_LEFT):
		w += 0.6
	if Input.is_physical_key_pressed(KEY_RIGHT):
		w -= 0.6
	drive_t += dt
	if (v != 0.0 or w != 0.0) and drive_t > 0.05:
		drive_t = 0.0
		_send({"type": "drive", "v": v, "w": w})

func _toggle_pause() -> void:
	_send({"type": "pause", "value": not st.get("paused", false)})

func _process(dt: float) -> void:
	_net()
	_drive(dt)
	var y: float = orbit["yaw"]
	var p: float = orbit["pitch"]
	var dir := Vector3(cos(p) * cos(y), sin(p), -cos(p) * sin(y))
	var tgt: Vector3 = orbit["target"]
	main_cam.look_at_from_position(tgt + dir * float(orbit["dist"]), tgt, Vector3.UP)
	var in_tun: bool = st.get("in_tunnel", false) and show["auto_roof"]
	main_cam.cull_mask = (1 | 4) if in_tun else (1 | 2 | 4)
	BeaconWorld.follow_weather(rain_main, main_cam)
	for k in ["pandar", "tele"]:
		pc_nodes[k].visible = show[k]
	radar_pts.visible = show["radar"]
	radar_boxes.visible = show["radar"]
	radar_edges.visible = show["radar"]
	gnss_mesh.visible = show["gnss"]
	trail_mesh.visible = show["gnss"]
	lane_mesh.visible = show["lanelet"]
	lbl_acc.visible = show["labels"] and world != null
	lbl_beacon.visible = show["labels"] and world != null
	# 카메라 패널: 카메라 주기(실시간 기준)마다 한 번 렌더
	cam_panel.visible = show["camera"] and sensor_sv != null and world != null
	if cam_panel.visible and not cam_busy:
		cam_next -= dt
		if cam_next <= 0.0:
			cam_next = 1.0 / float(hello.get("camera", {}).get("rate_hz", 10.0))
			BeaconWorld.follow_weather(rain_sensor, sensor_cam)
			sensor_sv.render_target_update_mode = SubViewport.UPDATE_ONCE
	if shot_path != "" and world != null and not st.is_empty():
		if shot_at < 0:
			shot_at = Time.get_ticks_msec() / 1000.0 + float(args.get("shot-delay", "8"))
		elif Time.get_ticks_msec() / 1000.0 > shot_at:
			var path := shot_path
			shot_path = ""
			await RenderingServer.frame_post_draw
			get_viewport().get_texture().get_image().save_png(path)
			_send({"type": "quit"})
			get_tree().quit()

# ================================================================== UI
func _load_fonts(fonts: Dictionary, icons: String) -> void:
	var fa := FontFile.new()
	fa.load_dynamic_font(icons)
	font_body = FontFile.new()
	font_body.load_dynamic_font(fonts["Medium"])
	font_body.fallbacks = [fa]
	font_bold = FontFile.new()
	font_bold.load_dynamic_font(fonts["Bold"])
	font_bold.fallbacks = [fa]
	ui.theme.default_font = font_body
	for l in [lbl_acc, lbl_beacon]:
		l.font = font_bold
	for k in dash:
		if dash[k] is Label and dash[k].get_meta("big", false):
			dash[k].add_theme_font_override("font", font_bold)

func _ic(code: int) -> String:
	return String.chr(code)

func _panel_style(bg: Color, radius := 12, pad := 12) -> StyleBoxFlat:
	var s := StyleBoxFlat.new()
	s.bg_color = bg
	s.set_corner_radius_all(radius)
	s.content_margin_left = pad
	s.content_margin_right = pad
	s.content_margin_top = pad * 0.7
	s.content_margin_bottom = pad * 0.7
	return s

func _build_ui() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	ui = Control.new()
	ui.set_anchors_preset(Control.PRESET_FULL_RECT)
	ui.mouse_filter = Control.MOUSE_FILTER_IGNORE
	layer.add_child(ui)
	var th := Theme.new()
	th.default_font_size = 15
	th.set_color("font_color", "Label", TEXT)
	th.set_color("font_color", "Button", TEXT)
	th.set_stylebox("panel", "PanelContainer", _panel_style(BG0))
	th.set_stylebox("normal", "Button", _panel_style(BG2, 8, 10))
	th.set_stylebox("hover", "Button", _panel_style(BG3, 8, 10))
	th.set_stylebox("pressed", "Button", _panel_style(Color(ACCENT, 0.85), 8, 10))
	th.set_stylebox("focus", "Button", StyleBoxEmpty.new())
	th.set_color("font_pressed_color", "Button", BG0)
	ui.theme = th
	_build_playbar()
	_build_dashboard()
	_build_log()
	_build_camera_panel()
	loading = Label.new()
	loading.text = "백엔드 연결 중..."
	loading.add_theme_font_size_override("font_size", 22)
	loading.add_theme_color_override("font_color", MUTED)
	loading.set_anchors_preset(Control.PRESET_CENTER)
	ui.add_child(loading)

func _btn(text: String, tip := "", toggle := false) -> Button:
	var b := Button.new()
	b.text = text
	b.tooltip_text = tip
	b.toggle_mode = toggle
	b.focus_mode = Control.FOCUS_NONE
	return b

func _chip(text: String, col: Color) -> Label:
	var l := Label.new()
	l.text = text
	l.add_theme_color_override("font_color", col)
	var s := _panel_style(Color(col, 0.16), 999, 8)
	s.content_margin_top = 3
	s.content_margin_bottom = 3
	l.add_theme_stylebox_override("normal", s)
	return l

func _build_playbar() -> void:
	var pc := PanelContainer.new()
	pc.set_anchors_preset(Control.PRESET_CENTER_TOP)
	pc.grow_horizontal = Control.GROW_DIRECTION_BOTH
	pc.position.y = 12
	pc.add_theme_stylebox_override("panel", _panel_style(BG0, 14, 12))
	ui.add_child(pc)
	var h := HBoxContainer.new()
	h.add_theme_constant_override("separation", 6)
	pc.add_child(h)
	btn_pause = _btn(_ic(I_PAUSE), "재생 / 일시정지 (Space)")
	btn_pause.add_theme_stylebox_override("normal", _panel_style(Color(ACCENT, 0.85), 8, 10))
	btn_pause.add_theme_color_override("font_color", BG0)
	btn_pause.pressed.connect(_toggle_pause)
	h.add_child(btn_pause)
	for r in [0.5, 1.0, 2.0, 4.0]:
		var b := _btn("%sx" % (str(r) if r < 1 else str(int(r))), "시뮬레이션 배속", true)
		b.pressed.connect(func(): _send({"type": "rtf", "value": r}))
		b.set_meta("rtf", r)
		speed_btns.append(b)
		h.add_child(b)
	var b1 := _btn(_ic(I_ROTATE), "시나리오 재시작 (R)")
	b1.pressed.connect(func(): _send({"type": "restart"}))
	h.add_child(b1)
	var b2 := _btn(_ic(I_MAP), "맵 재생성 (G)")
	b2.pressed.connect(func(): _send({"type": "restart", "map": true}))
	h.add_child(b2)
	lbl_time = _chip("", TEXT)
	lbl_env = _chip("", TEAL)
	lbl_cmd = _chip("", MUTED)
	lbl_ros = _chip("", MUTED)
	for l in [lbl_time, lbl_env, lbl_cmd, lbl_ros]:
		h.add_child(l)
	var sep := VSeparator.new()
	h.add_child(sep)
	for it in [["pandar", I_DOT, "P40"], ["tele", I_BULLSEYE, "Tele"], ["radar", I_WIFI, "레이더"],
			["gnss", I_SAT, "GNSS"], ["camera", I_CAMERA, ""]]:
		var key: String = it[0]
		var t := _btn(_ic(it[1]) + ("  " + it[2] if it[2] != "" else ""), "", true)
		t.button_pressed = show[key]
		t.toggled.connect(func(on): show[key] = on)
		toggles[key] = t
		h.add_child(t)

func _kpi(parent: Control, key: String, label: String, col: Color) -> void:
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 0)
	v.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var a := Label.new()
	a.text = label
	a.add_theme_color_override("font_color", MUTED)
	a.add_theme_font_size_override("font_size", 12)
	var b := Label.new()
	b.text = "-"
	b.add_theme_color_override("font_color", col)
	b.add_theme_font_size_override("font_size", 24)
	b.set_meta("big", true)
	v.add_child(a)
	v.add_child(b)
	parent.add_child(v)
	dash[key] = b

func _section(parent: Control, icon: int, title: String, col: Color) -> void:
	var l := Label.new()
	l.text = _ic(icon) + "  " + title
	l.add_theme_color_override("font_color", col)
	l.add_theme_font_size_override("font_size", 14)
	parent.add_child(l)

func _build_dashboard() -> void:
	var pc := PanelContainer.new()
	pc.set_anchors_preset(Control.PRESET_TOP_RIGHT)
	pc.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	pc.position = Vector2(-12, 68)
	pc.custom_minimum_size = Vector2(290, 0)
	ui.add_child(pc)
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 6)
	pc.add_child(v)
	_section(v, I_TOWER, "비콘", WARN)
	var r1 := HBoxContainer.new()
	v.add_child(r1)
	_kpi(r1, "v", "속도", TEXT)
	_kpi(r1, "acc", "사고 현장까지", AMBER)
	var g := Label.new()
	dash["gnss"] = g
	v.add_child(g)
	var dmg := Label.new()
	dmg.add_theme_color_override("font_color", DANGER)
	dash["damage"] = dmg
	v.add_child(dmg)
	v.add_child(HSeparator.new())
	_section(v, I_CAR, "접근 차량", ACCENT)
	var gr := GridContainer.new()
	gr.columns = 3
	v.add_child(gr)
	for it in [["active", "주행", TEXT], ["noticed", "비콘 인지", WARN], ["lane_change", "차로변경", ACCENT],
			["brake", "감속", TEAL], ["scc_lock", "SCC 인식", SCC], ["scc_follow", "SCC 추종", SCC]]:
		_kpi(gr, it[0], it[1], it[2])
	var cr := Label.new()
	dash["crash"] = cr
	v.add_child(cr)

func _build_log() -> void:
	var pc := PanelContainer.new()
	pc.set_anchors_preset(Control.PRESET_BOTTOM_LEFT)
	pc.grow_vertical = Control.GROW_DIRECTION_BEGIN
	pc.position = Vector2(12, -12)
	pc.custom_minimum_size = Vector2(460, 0)
	ui.add_child(pc)
	var v := VBoxContainer.new()
	pc.add_child(v)
	_section(v, I_INFO, "이벤트", MUTED)
	log_box = VBoxContainer.new()
	log_box.add_theme_constant_override("separation", 2)
	v.add_child(log_box)

func _add_log(line: String) -> void:
	log_lines.append(line)
	if log_lines.size() > 8:
		log_lines = log_lines.slice(log_lines.size() - 8)
	for c in log_box.get_children():
		c.queue_free()
	for ln in log_lines:
		var l := Label.new()
		l.text = ln
		l.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		l.custom_minimum_size = Vector2(436, 0)
		l.add_theme_font_size_override("font_size", 12)
		var col := TEXT
		if "추돌" in ln:
			col = DANGER
		elif "SCC" in ln:
			col = SCC
		elif "실패" in ln:
			col = WARN
		l.add_theme_color_override("font_color", col)
		log_box.add_child(l)

func _build_camera_panel() -> void:
	cam_panel = PanelContainer.new()
	cam_panel.set_anchors_preset(Control.PRESET_BOTTOM_RIGHT)
	cam_panel.grow_horizontal = Control.GROW_DIRECTION_BEGIN
	cam_panel.grow_vertical = Control.GROW_DIRECTION_BEGIN
	cam_panel.position = Vector2(-12, -12)
	cam_panel.add_theme_stylebox_override("panel", _panel_style(Color(BG1, 0.95), 12, 10))
	ui.add_child(cam_panel)
	var v := VBoxContainer.new()
	v.name = "v"
	cam_panel.add_child(v)
	var h := HBoxContainer.new()
	h.name = "h"
	v.add_child(h)
	h.add_child(_chip(_ic(I_CAMERA) + "  Arducam B0495", ACCENT))
	h.add_child(_chip("Godot", TEAL))
	var info := Label.new()
	info.name = "info"
	info.add_theme_color_override("font_color", MUTED)
	info.add_theme_font_size_override("font_size", 12)
	info.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(info)
	var x := _btn(_ic(I_XMARK))
	x.pressed.connect(func(): toggles["camera"].button_pressed = false)
	h.add_child(x)
	var tr := TextureRect.new()
	tr.name = "img"
	tr.custom_minimum_size = Vector2(500, 313)
	tr.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	tr.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_COVERED
	v.add_child(tr)

func _update_ui() -> void:
	var paused: bool = st.get("paused", false)
	btn_pause.text = _ic(I_PLAY if paused else I_PAUSE)
	for b in speed_btns:
		b.set_pressed_no_signal(absf(float(b.get_meta("rtf")) - float(st.get("rtf", 1.0))) < 1e-3)
	lbl_time.text = "%s  %.1f s · RTF %.2f" % [_ic(I_CLOCK), float(st["t"]), float(st["rtf_meas"])]
	lbl_env.text = _ic(I_INFO) + "  " + str(world_msg.get("env_label", ""))
	var src: String = st["cmd_src"]
	lbl_cmd.text = _ic(I_GAMEPAD) + "  명령 " + src
	lbl_cmd.add_theme_color_override("font_color", OK if src != "-" else MUTED)
	lbl_ros.text = _ic(I_LINK) + ("  ROS2" if st.get("ros", false) else "  ROS2 없음")
	lbl_ros.add_theme_color_override("font_color", ACCENT if st.get("ros", false) else MUTED)
	dash["v"].text = "%+.1f m/s" % float(st["v"])
	dash["acc"].text = "%.0f m" % float(st["acc_dist"])
	var g = st.get("gnss")
	if typeof(g) == TYPE_DICTIONARY:
		dash["gnss"].text = "%s  GNSS %s · σ %.2f m" % [_ic(I_SAT), g["name"], float(g["hacc"])]
		dash["gnss"].add_theme_color_override("font_color", Color(g["color"][0], g["color"][1], g["color"][2]))
	dash["damage"].text = "비콘 파손 — 명령 무시" if st.get("damaged", false) else (
		("접촉: " + str(st["contact"])) if st.get("contact") != null else "")
	var c: Dictionary = st["counts"]
	dash["active"].text = str(int(st["n_active"]))
	for k in ["noticed", "lane_change", "brake", "scc_lock"]:
		dash[k].text = str(int(c.get(k, 0)))
	dash["scc_follow"].text = str(int(st["scc_follow"]))
	var crash := int(c.get("crash", 0))
	dash["crash"].text = "%s  추돌 %d · 비콘 추돌 %d" % [_ic(I_BURST), crash, int(c.get("beacon_hit", 0))]
	dash["crash"].add_theme_color_override("font_color", DANGER if crash else OK)
