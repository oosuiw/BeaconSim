extends Node3D
## BeaconSim 앱 (Godot 프런트엔드). 백엔드(beacon_sim/server.py)와 TCP 로 연결.
## 화면: 3D 월드(고품질) + 센서 표시 + 사이드바(설정 탭·이벤트) / 재생 바 / 상황판 / 카메라 패널 / 도움말.
## 조작: 마우스 시점, 방향키 비콘 주행, 단축키 (F1 도움말).

const BeaconWorld = preload("res://world.gd")
const K = preload("res://ui_kit.gd")
const Sidebar = preload("res://sidebar.gd")

const SENSOR_MASK := 1 | 2 | 8 | 16 | 32      # 센서 카메라: 오버레이(4) 빼고 전부
const VIEW_KEYS := ["pandar", "tele", "radar", "radar_fov", "gnss", "camera", "labels", "vehicles", "scc", "robot",
	"markers", "lanelet", "terrain", "tunnel", "auto_roof", "env_view", "legend"]

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
var show := {"pandar": true, "tele": true, "radar": true, "radar_fov": false, "gnss": true, "camera": true,
	"labels": true, "vehicles": true, "scc": true, "robot": true, "markers": true, "lanelet": false, "terrain": true,
	"tunnel": true, "auto_roof": true, "env_view": true, "legend": true}
var pandar_color := 0
var ui_zoom := 0.0
var veh_dims: Array = []
var robot_T := Transform3D()
var drive_t := 0.0
var cam_next := 0.0
var cam_busy := false
# 오버레이
var pc_nodes := {}
var pandar_last := {}
var radar_pts: MeshInstance3D
var radar_boxes: MultiMeshInstance3D
var radar_edges: MultiMeshInstance3D
var fan_node: Node3D
var fan_local := Transform3D()
var gnss_mesh: MeshInstance3D
var gnss_trail := PackedVector3Array()
var gnss_trail_col := PackedColorArray()
var truth_trail := PackedVector3Array()
var trail_mesh: MeshInstance3D
var last_gnss_t := -1.0
var scc_mesh: MeshInstance3D
var scc_lines: MeshInstance3D
var lane_mesh: MeshInstance3D
var acc_mesh: MeshInstance3D
var lbl_acc: Label3D
var lbl_beacon: Label3D
var veh_lbls: Array = []
# UI
var font_body: FontFile
var font_bold: FontFile
var ui: Control
var sidebar: PanelContainer
var loading: Label
var playbar: PanelContainer
var btn_pause: Button
var speed_btns: Array = []
var lbl_time: Label
var lbl_env: Label
var lbl_cmd: Label
var lbl_ros: Label
var toggles := {}
var dash := {}
var dash_panel: PanelContainer
var legend_box: VBoxContainer
var cam_panel: PanelContainer
var help_layer: Control
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
	if args.has("selftest"):
		var t: Node = preload("res://selftest.gd").new()
		add_child(t)
		t.run(self, args["selftest"])

func _notification(what: int) -> void:
	if what == NOTIFICATION_WM_CLOSE_REQUEST:
		_send({"type": "quit"})
		get_tree().quit()

func _on_resize() -> void:
	var s := get_window().size
	var z := ui_zoom
	if z <= 0.0:
		z = clampf(minf(s.y / 1000.0, s.x / 1700.0), 0.75, 3.0)
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
			sidebar.set_version(str(m["version"]))
		"ui":
			sidebar.set_tabs(m["tabs"])
		"values":
			sidebar.set_values(m["values"])
			_apply_view()
		"info":
			sidebar.set_info(m["info"])
		"world":
			_on_world(m, bin)
		"pc":
			_on_pc(m, bin)
		"radar":
			_on_radar(m, bin)
		"log":
			for ln in m["lines"]:
				sidebar.add_log(ln)
		"cam_req":
			_capture_for_ros(m)
		"bye":
			get_tree().quit()

# ================================================================== 설정 값 (사이드바·재생 바 → 백엔드)
func _on_edit(k: String, v: Variant) -> void:
	if k == "view.ui_zoom_auto" and not v:                  # 자동 끔 → 지금 배율에서 시작
		var cur := get_window().content_scale_factor
		_send({"type": "set", "k": k, "v": v})
		sidebar.set_widget(k, v)
		k = "view.ui_zoom"
		v = snappedf(cur, 0.05)
	sidebar.set_widget(k, v)
	if k == "view.follow":
		follow = bool(v)
		return
	_send({"type": "set", "k": k, "v": v})
	if k.begins_with("view."):
		_apply_view()

func _on_action(name: String) -> void:
	if name.begins_with("preset:"):
		_preset(name.trim_prefix("preset:"))
		return
	_send({"type": "action", "name": name})

## 화면 표시 값(view.*)을 3D·UI 에 반영
func _apply_view() -> void:
	var vals: Dictionary = sidebar.vals
	for k in VIEW_KEYS:
		if vals.has("view." + k):
			show[k] = bool(vals["view." + k])
	for k in toggles:
		toggles[k].set_pressed_no_signal(show[k])
	var pc := int(vals.get("view.pandar_color", 0))
	if pc != pandar_color:
		pandar_color = pc
		_show_pandar()
	var z := 0.0 if vals.get("view.ui_zoom_auto", true) else float(vals.get("view.ui_zoom", 0.0))
	if z != ui_zoom:
		ui_zoom = z
		_on_resize()
	if int(vals.get("view.log_filter", 0)) != sidebar.feed_filter:
		sidebar._set_feed_filter(int(vals.get("view.log_filter", 0)), false)
	sidebar.set_widget("view.follow", follow)
	legend_box.visible = show["legend"]
	_set_main_env()

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
	pandar_last = {}
	gnss_trail = PackedVector3Array()
	gnss_trail_col = PackedColorArray()
	truth_trail = PackedVector3Array()
	last_gnss_t = -1.0
	if m.has("arrays") and m["arrays"].has("lane_pts"):
		var p := _arr(m, bin, "lane_pts").to_vector3_array()
		var c := _arr(m, bin, "lane_cols").to_color_array()
		lane_mesh.mesh = _prim_mesh(Mesh.PRIMITIVE_LINES, p, c)
	if m.has("arrays") and m["arrays"].has("fan_tri") and m.get("radar_T") != null:
		_build_fan(_arr(m, bin, "fan_tri").to_vector3_array(), _arr(m, bin, "fan_col").to_color_array(),
			_arr(m, bin, "fan_edge").to_vector3_array())
		fan_local = BeaconWorld.xf(m["radar_T"])
	var a: Array = m["accident"]
	var ap := Vector3(a[0], a[1], a[2])
	acc_mesh.mesh = _prim_mesh(Mesh.PRIMITIVE_LINES, PackedVector3Array([ap, ap + Vector3(0, 6, 0)]),
		PackedColorArray([K.AMBER, K.AMBER]))
	lbl_acc.position = ap + Vector3(0, 6.6, 0)
	loading.visible = false
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
	var idx := _arr(m, bin, "veh_idx").to_int32_array()
	var T := _arr(m, bin, "veh_T").to_float32_array()
	world.set_vehicles(idx, T)
	if sensor_cam:
		sensor_cam.transform = BeaconWorld.xf(m["cam"])
	fan_node.transform = robot_T * fan_local
	_update_scc(idx, T, _arr(m, bin, "veh_flags"), _arr(m, bin, "veh_kmh").to_float32_array())
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

func _line_mat(alpha := false) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.vertex_color_use_as_albedo = true
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

func _mi(mat: Material, parent: Node = null) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	mi.material_override = mat
	mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	mi.layers = 4                         # 레이어 3: 센서 카메라에는 안 보임
	(parent if parent else self).add_child(mi)
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
	em.surface_set_material(0, _line_mat())
	radar_edges = MultiMeshInstance3D.new()
	radar_edges.layers = 4
	radar_edges.multimesh = MultiMesh.new()
	radar_edges.multimesh.transform_format = MultiMesh.TRANSFORM_3D
	radar_edges.multimesh.mesh = em
	add_child(radar_edges)
	fan_node = Node3D.new()
	add_child(fan_node)
	gnss_mesh = _mi(_line_mat())
	trail_mesh = _mi(_pt_mat(3.0))
	scc_mesh = _mi(_pt_mat(9.0))
	scc_lines = _mi(_line_mat())
	lane_mesh = _mi(_line_mat())
	acc_mesh = _mi(_line_mat())
	lbl_acc = _label3d("사고 현장", K.AMBER)
	lbl_beacon = _label3d("비콘", K.WARN)
	for i in range(24):
		var l := _label3d("", K.SCC)
		l.visible = false
		veh_lbls.append(l)

## 레이더 감지 범위 면 (레이더 좌표 → fan_node 아래, 매 상태마다 비콘 자세로 옮김)
func _build_fan(tri: PackedVector3Array, col: PackedColorArray, edge: PackedVector3Array) -> void:
	for c in fan_node.get_children():
		c.queue_free()
	var fm := _line_mat(true)
	fm.cull_mode = BaseMaterial3D.CULL_DISABLED
	fm.albedo_color = Color(1, 1, 1, 0.55)
	var f := _mi(fm, fan_node)
	f.mesh = _prim_mesh(Mesh.PRIMITIVE_TRIANGLES, tri, col)
	var ec := PackedColorArray()
	ec.resize(edge.size())
	ec.fill(Color(1.0, 0.40, 0.40, 0.8))
	var e := _mi(_line_mat(true), fan_node)
	e.mesh = _prim_mesh(Mesh.PRIMITIVE_LINES, edge, ec)

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
	add_child(l)
	return l

func _on_pc(m: Dictionary, bin: PackedByteArray) -> void:
	var name: String = m["name"]
	if not pc_nodes.has(name):
		return
	var xyz := _arr(m, bin, "xyz").to_vector3_array()
	var col := _arr(m, bin, "col").to_color_array()
	if name == "pandar":
		pandar_last = {"xyz": xyz, "col": col, "col2": _arr(m, bin, "col2").to_color_array()}
		_show_pandar()
		return
	pc_nodes[name].mesh = _prim_mesh(Mesh.PRIMITIVE_POINTS, xyz, col)

func _show_pandar() -> void:
	if pandar_last.is_empty():
		return
	pc_nodes["pandar"].mesh = _prim_mesh(Mesh.PRIMITIVE_POINTS, pandar_last["xyz"],
		pandar_last["col2"] if pandar_color == 1 else pandar_last["col"])

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

## SCC·운전자 상태 점 + SCC 추종 선 + 차량 이름표 (추돌 / SCC 추종 속도)
func _update_scc(idx: PackedInt32Array, T: PackedFloat32Array, fl: PackedByteArray, kmh: PackedFloat32Array) -> void:
	var pts := PackedVector3Array()
	var cols := PackedColorArray()
	var lines := PackedVector3Array()
	var bpos := robot_T.origin + Vector3(0, 0.5, 0)
	var nl := 0
	var lbl_on: bool = show["labels"] and show["vehicles"]
	for k in range(idx.size()):
		var f := fl[k]
		var tr := BeaconWorld.xf_at(T, k)
		var d: Array = veh_dims[idx[k]]
		# 이름표: 비콘 300 m 안
		if lbl_on and nl < veh_lbls.size() and (f & 3) and tr.origin.distance_to(robot_T.origin) < 300.0:
			var l: Label3D = veh_lbls[nl]
			nl += 1
			var crashed := (f & 1) != 0
			l.text = "●  " + ("추돌" if crashed else "SCC 추종 %.0f km/h" % kmh[k])
			l.modulate = K.DANGER if crashed else K.SCC
			l.position = tr.origin + tr.basis.y * (float(d[2]) + 1.4)
			l.visible = true
		if not (f & 16):
			continue
		var col: Color
		if f & 1:
			col = K.DANGER
		elif f & 2:
			col = K.SCC
		elif f & 4:
			col = K.WARN
		elif f & 8:
			col = K.TEAL
		else:
			continue
		pts.append(tr.origin + tr.basis.y * (float(d[2]) + 0.6))
		cols.append(col)
		if f & 2:
			lines.append(tr.origin + tr.basis.x * (float(d[0]) / 2.0) + tr.basis.y * 0.8)
			lines.append(bpos)
	for i in range(nl, veh_lbls.size()):
		veh_lbls[i].visible = false
	scc_mesh.mesh = _prim_mesh(Mesh.PRIMITIVE_POINTS, pts, cols)
	var lc := PackedColorArray()
	lc.resize(lines.size())
	lc.fill(K.SCC)
	scc_lines.mesh = _prim_mesh(Mesh.PRIMITIVE_LINES, lines, lc)

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
	sensor_cam.cull_mask = SENSOR_MASK
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
	sidebar.set_widget("view.follow", follow)

func _set_follow(on: bool) -> void:
	follow = on
	sidebar.set_widget("view.follow", follow)

func _unhandled_input(ev: InputEvent) -> void:
	if ev is InputEventMouseButton and ev.pressed:
		get_viewport().gui_release_focus()
	if help_layer.visible:
		if ev is InputEventKey and ev.pressed and (ev as InputEventKey).physical_keycode in [KEY_ESCAPE, KEY_F1]:
			help_layer.visible = false
		return
	if ev is InputEventMouseMotion:
		var mm := ev as InputEventMouseMotion
		var d := mm.relative
		if mm.button_mask & MOUSE_BUTTON_MASK_LEFT:
			orbit["yaw"] -= d.x * 0.005
			orbit["pitch"] = clampf(orbit["pitch"] + d.y * 0.005, deg_to_rad(2), deg_to_rad(89))
		elif mm.button_mask & (MOUSE_BUTTON_MASK_RIGHT | MOUSE_BUTTON_MASK_MIDDLE):
			_set_follow(false)
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
				_on_action("restart")
			KEY_G:
				_on_action("regen_map")
			KEY_F:
				_set_follow(not follow)
			KEY_1:
				_preset("beacon")
			KEY_2:
				_preset("overview")
			KEY_3:
				_preset("tunnel")
			KEY_F1:
				help_layer.visible = true

## 방향키: 단위 명령(-1~1)을 보내고 백엔드가 방향키 속도·회전 설정을 곱한다
func _drive(dt: float) -> void:
	if get_viewport().gui_get_focus_owner() != null or help_layer.visible:
		return
	var v := 0.0
	var w := 0.0
	if Input.is_physical_key_pressed(KEY_UP):
		v += 1.0
	if Input.is_physical_key_pressed(KEY_DOWN):
		v -= 1.0
	if Input.is_physical_key_pressed(KEY_LEFT):
		w += 1.0
	if Input.is_physical_key_pressed(KEY_RIGHT):
		w -= 1.0
	drive_t += dt
	if (v != 0.0 or w != 0.0) and drive_t > 0.05:
		drive_t = 0.0
		_send({"type": "drive", "v": v, "w": w})

func _toggle_pause() -> void:
	_send({"type": "pause", "value": not st.get("paused", false)})

func _process(dt: float) -> void:
	_net()
	_drive(dt)
	_layout()
	var y: float = orbit["yaw"]
	var p: float = orbit["pitch"]
	var dir := Vector3(cos(p) * cos(y), sin(p), -cos(p) * sin(y))
	var tgt: Vector3 = orbit["target"]
	main_cam.look_at_from_position(tgt + dir * float(orbit["dist"]), tgt, Vector3.UP)
	var in_tun: bool = st.get("in_tunnel", false) and show["auto_roof"]
	var mask := 1 | 4
	if not in_tun:
		mask |= (2 if show["tunnel"] else 0) | (8 if show["terrain"] else 0)
	mask |= (16 if show["vehicles"] else 0) | (32 if show["robot"] else 0)
	main_cam.cull_mask = mask
	BeaconWorld.follow_weather(rain_main, main_cam)
	for k in ["pandar", "tele"]:
		pc_nodes[k].visible = show[k]
	radar_pts.visible = show["radar"]
	radar_boxes.visible = show["radar"]
	radar_edges.visible = show["radar"]
	fan_node.visible = show["radar_fov"] and world != null
	gnss_mesh.visible = show["gnss"]
	trail_mesh.visible = show["gnss"]
	lane_mesh.visible = show["lanelet"]
	acc_mesh.visible = show["markers"]
	lbl_acc.visible = show["labels"] and show["markers"] and world != null
	lbl_beacon.visible = show["labels"] and world != null
	scc_mesh.visible = show["scc"]
	scc_lines.visible = show["scc"]
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
			if args.has("shot-tab"):
				sidebar.select_tab(int(args["shot-tab"]))
			if args.has("shot-help"):
				help_layer.visible = true
			await get_tree().create_timer(0.5).timeout
			await RenderingServer.frame_post_draw
			get_viewport().get_texture().get_image().save_png(path)
			_send({"type": "quit"})
			get_tree().quit()

## 사이드바 오른쪽 영역 기준으로 재생 바 가운데 정렬, 패널이 사이드바를 가리지 않게
func _layout() -> void:
	var vs := get_viewport().get_visible_rect().size
	var sw: float = Sidebar.WIDTH
	sidebar.position = Vector2.ZERO
	sidebar.size = Vector2(sw, vs.y)
	playbar.size = Vector2.ZERO
	playbar.position = Vector2(maxf(sw + (vs.x - sw - playbar.size.x) / 2.0, sw + 8), 12)
	dash_panel.size = Vector2(dash_panel.custom_minimum_size.x, 0)
	dash_panel.position = Vector2(vs.x - dash_panel.size.x - 12, playbar.position.y + playbar.size.y + 12)
	cam_panel.position = Vector2(maxf(vs.x - cam_panel.size.x - 12, sw + 12), vs.y - cam_panel.size.y - 12)
	loading.position = Vector2(sw + (vs.x - sw - loading.size.x) / 2.0, vs.y / 2.0)

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
	for l in [lbl_acc, lbl_beacon] + veh_lbls:
		l.font = font_bold
	for k in dash:
		if dash[k] is Label and dash[k].get_meta("big", false):
			dash[k].add_theme_font_override("font", font_bold)
	sidebar.set_fonts(font_bold)
	_apply_bold(help_layer)

func _apply_bold(n: Node) -> void:
	for c in n.get_children():
		if c is Label and c.get_meta("bold", false):
			c.add_theme_font_override("font", font_bold)
		_apply_bold(c)

func _build_ui() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	ui = Control.new()
	ui.set_anchors_preset(Control.PRESET_FULL_RECT)
	ui.mouse_filter = Control.MOUSE_FILTER_IGNORE
	layer.add_child(ui)
	ui.theme = K.make_theme()
	sidebar = Sidebar.new()
	sidebar.edited.connect(_on_edit)
	sidebar.action.connect(_on_action)
	ui.add_child(sidebar)
	_build_playbar()
	_build_dashboard()
	_build_camera_panel()
	loading = K.label("백엔드 연결 중...", K.MUTED, 22)
	ui.add_child(loading)
	_build_help()

func _build_playbar() -> void:
	playbar = PanelContainer.new()
	K.eat_mouse(playbar)
	playbar.add_theme_stylebox_override("panel", K.panel_style(K.BG0, 14, 12))
	ui.add_child(playbar)
	var h := HBoxContainer.new()
	h.add_theme_constant_override("separation", 6)
	playbar.add_child(h)
	btn_pause = K.primary(K.btn(K.ic(K.I_PAUSE), "재생 / 일시정지 (Space)"))
	btn_pause.pressed.connect(_toggle_pause)
	h.add_child(btn_pause)
	for r in [0.5, 1.0, 2.0, 4.0]:
		var b := K.btn("%sx" % (str(r) if r < 1 else str(int(r))), "시뮬레이션 배속", true)
		b.pressed.connect(func(): _send({"type": "rtf", "value": r}))
		b.set_meta("rtf", r)
		speed_btns.append(b)
		h.add_child(b)
	var b1 := K.btn(K.ic(K.I_ROTATE), "시나리오 재시작 (R)")
	b1.pressed.connect(func(): _on_action("restart"))
	h.add_child(b1)
	var b2 := K.btn(K.ic(K.I_MAP), "맵 재생성 (G)")
	b2.pressed.connect(func(): _on_action("regen_map"))
	h.add_child(b2)
	lbl_time = K.chip("", K.TEXT)
	lbl_env = K.chip("", K.TEAL)
	lbl_cmd = K.chip("", K.MUTED)
	lbl_cmd.tooltip_text = "비콘 이동 명령 출처: /cmd_vel(상위 제어기) 우선, 없으면 방향키"
	lbl_cmd.mouse_filter = Control.MOUSE_FILTER_PASS
	lbl_ros = K.chip("", K.MUTED)
	for l in [lbl_time, lbl_env, lbl_cmd, lbl_ros]:
		h.add_child(l)
	h.add_child(VSeparator.new())
	for it in [["pandar", K.I_DOT, "P40"], ["tele", K.I_BULLSEYE, "Tele"], ["radar", K.I_WIFI, "레이더"],
			["gnss", K.I_SAT, "GNSS"], ["camera", K.I_CAMERA, ""]]:
		var key: String = it[0]
		var t := K.btn(K.ic(it[1]) + ("  " + it[2] if it[2] != "" else ""), "", true)
		t.button_pressed = show[key]
		t.toggled.connect(func(on): _on_edit("view." + key, on))
		toggles[key] = t
		h.add_child(t)
	var hb := K.btn(K.ic(K.I_QUESTION), "조작 도움말 (F1)")
	hb.pressed.connect(func(): help_layer.visible = true)
	h.add_child(hb)

func _kpi(parent: Control, key: String, label: String, col: Color) -> void:
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 0)
	v.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var a := K.label(label, K.MUTED, 12)
	var b := K.label("-", col, 24)
	b.set_meta("big", true)
	v.add_child(a)
	v.add_child(b)
	parent.add_child(v)
	dash[key] = b

func _build_dashboard() -> void:
	dash_panel = PanelContainer.new()
	dash_panel.custom_minimum_size = Vector2(300, 0)
	ui.add_child(dash_panel)
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 6)
	dash_panel.add_child(v)
	K.section(v, K.I_TOWER, "비콘", K.WARN)
	var r1 := HBoxContainer.new()
	v.add_child(r1)
	_kpi(r1, "v", "속도", K.TEXT)
	_kpi(r1, "acc", "사고 현장까지", K.AMBER)
	var dmg := K.chip("", K.DANGER)
	dash["damage"] = dmg
	v.add_child(dmg)
	var g := K.chip("", K.OK)
	dash["gnss"] = g
	v.add_child(g)
	v.add_child(HSeparator.new())
	K.section(v, K.I_CAR, "접근 차량", K.ACCENT)
	var gr := GridContainer.new()
	gr.columns = 3
	v.add_child(gr)
	for it in [["active", "주행", K.TEXT], ["noticed", "비콘 인지", K.WARN], ["lane_change", "차로변경", K.ACCENT],
			["brake", "감속", K.TEAL], ["scc_lock", "SCC 인식", K.SCC], ["scc_follow", "SCC 추종", K.SCC]]:
		_kpi(gr, it[0], it[1], it[2])
	var cr := K.chip("", K.OK)
	dash["crash"] = cr
	v.add_child(cr)
	legend_box = VBoxContainer.new()
	legend_box.add_theme_constant_override("separation", 2)
	v.add_child(legend_box)
	legend_box.add_child(HSeparator.new())
	legend_box.add_child(K.label(K.ic(K.I_INFO) + "  범례", K.MUTED, 13))
	for it in [[K.TEAL, "SCC 작동 (접근 차량)"], [K.SCC, "SCC가 비콘 추종 (비콘까지 선)"], [K.WARN, "운전자가 비콘 인지"],
			[K.DANGER, "추돌"], [Color(1, 0.3, 0.3), "레이더 감지 범위(면) · 클러스터"],
			[K.AMBER, "레이더 감지 객체 (앞면 사각 판) · 사고 현장"], [Color(1, 0.62, 0.25), "Tele-15"],
			[K.ACCENT, "Pandar40P (세기)"], [Color.WHITE, "GNSS 정답 궤적 / 측정(상태색)"]]:
		var h := HBoxContainer.new()
		h.add_child(K.label(K.ic(K.I_CIRCLE), it[0], 10))
		h.add_child(K.label(it[1], K.TEXT, 12))
		legend_box.add_child(h)
	_ignore_mouse(dash_panel)

func _ignore_mouse(n: Node) -> void:
	if n is Control:
		(n as Control).mouse_filter = Control.MOUSE_FILTER_IGNORE
	for c in n.get_children():
		_ignore_mouse(c)

func _build_camera_panel() -> void:
	cam_panel = PanelContainer.new()
	K.eat_mouse(cam_panel)
	cam_panel.add_theme_stylebox_override("panel", K.panel_style(Color(K.BG1, 0.95), 12, 10))
	ui.add_child(cam_panel)
	var v := VBoxContainer.new()
	v.name = "v"
	cam_panel.add_child(v)
	var h := HBoxContainer.new()
	h.name = "h"
	v.add_child(h)
	h.add_child(K.chip(K.ic(K.I_CAMERA) + "  Arducam B0495", K.ACCENT))
	h.add_child(K.chip("Godot", K.TEAL))
	var info := K.label("", K.MUTED, 12)
	info.name = "info"
	info.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(info)
	var x := K.btn(K.ic(K.I_XMARK))
	x.pressed.connect(func(): _on_edit("view.camera", false))
	h.add_child(x)
	var tr := TextureRect.new()
	tr.name = "img"
	tr.custom_minimum_size = Vector2(500, 313)
	tr.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	tr.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_COVERED
	v.add_child(tr)

func _build_help() -> void:
	help_layer = Control.new()
	help_layer.set_anchors_preset(Control.PRESET_FULL_RECT)
	help_layer.visible = false
	ui.add_child(help_layer)
	var dim := ColorRect.new()
	dim.color = Color(0, 0, 0, 0.45)
	dim.set_anchors_preset(Control.PRESET_FULL_RECT)
	dim.gui_input.connect(func(e): if e is InputEventMouseButton and e.pressed: help_layer.visible = false)
	help_layer.add_child(dim)
	var cc := CenterContainer.new()
	cc.set_anchors_preset(Control.PRESET_FULL_RECT)
	cc.mouse_filter = Control.MOUSE_FILTER_IGNORE
	help_layer.add_child(cc)
	var pc := PanelContainer.new()
	pc.add_theme_stylebox_override("panel", K.panel_style(K.BG1, 14, 20, K.BORDER))
	pc.custom_minimum_size = Vector2(560, 0)
	K.eat_mouse(pc)
	cc.add_child(pc)
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 10)
	pc.add_child(v)
	var t := K.label(K.ic(K.I_QUESTION) + "  조작 도움말", K.TEXT, 20)
	t.set_meta("bold", true)
	v.add_child(t)
	var gr := GridContainer.new()
	gr.columns = 2
	gr.add_theme_constant_override("h_separation", 16)
	gr.add_theme_constant_override("v_separation", 6)
	v.add_child(gr)
	for it in [["↑ / ↓", "비콘 전진 / 후진 (ROS /cmd_vel이 오면 그쪽 우선)"], ["← / →", "비콘 회전"],
			["좌클릭 드래그", "시점 회전"], ["우클릭 드래그", "시점 이동 (따라가기 해제)"], ["휠", "확대 / 축소"],
			["F", "비콘 따라가기"], ["1  2  3", "비콘 / 전체 / 터널 시점"], ["Space", "일시정지 / 재생"],
			["R", "시나리오 재시작"], ["G", "맵 재생성"], ["F1", "도움말"]]:
		var c := K.chip(it[0], K.ACCENT)
		c.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
		c.custom_minimum_size = Vector2(130, 0)
		gr.add_child(c)
		gr.add_child(K.label(it[1]))
	v.add_child(K.label("주황 기둥 = 사고 현장,  비콘 위 짧은 선 = GNSS 위치 (색 = RTK 상태)", K.MUTED, 13))
	var b := K.primary(K.btn("닫기"))
	b.pressed.connect(func(): help_layer.visible = false)
	v.add_child(b)

func _update_ui() -> void:
	var paused: bool = st.get("paused", false)
	btn_pause.text = K.ic(K.I_PLAY if paused else K.I_PAUSE)
	for b in speed_btns:
		b.set_pressed_no_signal(absf(float(b.get_meta("rtf")) - float(st.get("rtf", 1.0))) < 1e-3)
	lbl_time.text = "%s  %.1f s · RTF %.2f" % [K.ic(K.I_CLOCK), float(st["t"]), float(st["rtf_meas"])]
	lbl_env.text = K.ic(_env_icon()) + "  " + str(world_msg.get("env_label", ""))
	var src: String = st["cmd_src"]
	lbl_cmd.text = K.ic(K.I_GAMEPAD) + "  명령 " + src
	K.set_chip_color(lbl_cmd, K.OK if src != "-" else K.MUTED)
	var ros: bool = st.get("ros", false)
	lbl_ros.text = K.ic(K.I_LINK) + ("  ROS2" if ros else "  ROS2 없음")
	K.set_chip_color(lbl_ros, K.ACCENT if ros else K.MUTED)
	dash["v"].text = "%+.1f m/s" % float(st["v"])
	dash["acc"].text = "%.0f m" % float(st["acc_dist"])
	var g = st.get("gnss")
	dash["gnss"].visible = typeof(g) == TYPE_DICTIONARY
	if typeof(g) == TYPE_DICTIONARY:
		dash["gnss"].text = "%s  GNSS %s · σ %.2f m" % [K.ic(K.I_SAT), g["name"], float(g["hacc"])]
		var gc := {4: K.OK, 5: K.WARN, 1: K.AMBER, 0: K.DANGER}
		K.set_chip_color(dash["gnss"], gc.get(int(g["status"]), K.MUTED))
	if st.get("damaged", false):
		dash["damage"].text = K.ic(K.I_BURST) + "  비콘 파손 — 명령 무시"
		K.set_chip_color(dash["damage"], K.DANGER)
		dash["damage"].visible = true
	elif st.get("contact") != null:
		dash["damage"].text = K.ic(K.I_HAND) + "  접촉: " + str(st["contact"])
		K.set_chip_color(dash["damage"], K.WARN)
		dash["damage"].visible = true
	else:
		dash["damage"].visible = false
	var c: Dictionary = st["counts"]
	dash["active"].text = str(int(st["n_active"]))
	for k in ["noticed", "lane_change", "brake", "scc_lock"]:
		dash[k].text = str(int(c.get(k, 0)))
	dash["scc_follow"].text = str(int(st["scc_follow"]))
	var crash := int(c.get("crash", 0))
	dash["crash"].text = "%s  추돌 %d  ·  비콘 추돌 %d" % [K.ic(K.I_BURST if crash else K.I_CHECK), crash,
		int(c.get("beacon_hit", 0))]
	K.set_chip_color(dash["crash"], K.DANGER if crash else K.OK)

func _env_icon() -> int:
	var e = world_msg.get("env", {})
	if typeof(e) != TYPE_DICTIONARY:
		return K.I_SUN
	var wx: String = e.get("weather", "")
	if wx == "rain":
		return K.I_RAIN
	if wx == "snow":
		return K.I_SNOW
	if wx == "fog":
		return K.I_SMOG
	var tod: String = e.get("time_of_day", "")
	return {"night": K.I_MOON, "dusk": K.I_CLOUD_SUN}.get(tod, K.I_SUN)
