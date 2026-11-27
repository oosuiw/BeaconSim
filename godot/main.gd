extends Node3D
## BeaconSim 카메라 렌더러 (Godot 4 Forward+ 시제품)
## 실행: godot --path godot -- --scene=<scene.json> --port=47810
## 파이썬(beacon_sim/godot_link.py)이 UDP JSON 으로 상태를 보내고, 캡처 요청이면 PNG 를 쓰고 응답한다.

var info: Dictionary
var udp := PacketPeerUDP.new()
var reply := PacketPeerUDP.new()
var sv: SubViewport
var cam: Camera3D
var world_env: WorldEnvironment
var sun: DirectionalLight3D
var robot: Node3D
var protos: Array = []
var slots: Array = []          # 슬롯별 차량 인스턴스
var slot_lights: Array = []
var acc_lights: Array = []
var shown := {}
var rain: GPUParticles3D
var pending_capture := ""
var pending_shm := false
var pending_seq := 0
var wait_frames := 0
var label: Label
var mats := {}
var lamps_on := false

func _ready() -> void:
	var args := {}
	for a in OS.get_cmdline_user_args():
		var kv: PackedStringArray = a.trim_prefix("--").split("=", true, 1)
		args[kv[0]] = kv[1] if kv.size() > 1 else ""
	var port := int(args.get("port", "47810"))
	var reply_port := int(args.get("reply", str(port + 1)))
	var hidden := args.has("hidden")
	if hidden:   # GUI 연동: 최소화하면 렌더링이 멈추고 화면 밖 좌표는 창 관리자가 무시 → 1x1 투명 창, 클릭 통과
		get_viewport().transparent_bg = true
		DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_BORDERLESS, true)
		DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_TRANSPARENT, true)
		DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_NO_FOCUS, true)
		DisplayServer.window_set_mouse_passthrough(PackedVector2Array([Vector2(0, 0), Vector2(0, 0), Vector2(0, 0)]))
		DisplayServer.window_set_size(Vector2i(1, 1))
		DisplayServer.window_set_title("BeaconSim 카메라 렌더러")
		Engine.max_fps = 60
	var js_path: String = args.get("scene", "")
	info = JSON.parse_string(FileAccess.get_file_as_string(js_path))
	_build_viewport(hidden)
	_load_scene(js_path.get_base_dir().path_join("scene.glb"))
	_build_env(info["env"])
	_build_lights(info)
	_build_weather(info["env"])
	udp.bind(port, "127.0.0.1")
	reply.set_dest_address("127.0.0.1", reply_port)
	# 노이즈 텍스처(비동기 생성) 준비 대기 후 준비 완료 알림
	await get_tree().create_timer(1.0).timeout
	_send({"type": "ready"})

func _send(d: Dictionary) -> void:
	reply.put_packet(JSON.stringify(d).to_utf8_buffer())

# ------------------------------------------------------------------ 카메라 (오버스캔 핀홀, 왜곡은 파이썬에서)
func _build_viewport(hidden := false) -> void:
	var c: Dictionary = info["camera"]
	sv = SubViewport.new()
	sv.size = Vector2i(int(c["width"]), int(c["height"]))
	sv.msaa_3d = Viewport.MSAA_4X
	sv.screen_space_aa = Viewport.SCREEN_SPACE_AA_DISABLED
	sv.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	sv.positional_shadow_atlas_size = 4096
	add_child(sv)
	cam = Camera3D.new()
	cam.keep_aspect = Camera3D.KEEP_HEIGHT
	cam.fov = float(c["vfov_deg"])
	cam.near = max(float(c["near"]), 0.1)
	cam.far = 3000.0
	cam.current = true
	sv.add_child(cam)
	var tr := TextureRect.new()
	tr.texture = sv.get_texture()
	tr.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	tr.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	tr.set_anchors_preset(Control.PRESET_FULL_RECT)
	var layer := CanvasLayer.new()
	layer.visible = not hidden
	add_child(layer)
	layer.add_child(tr)
	label = Label.new()
	label.position = Vector2(12, 8)
	label.add_theme_color_override("font_color", Color(1, 1, 1, 0.85))
	layer.add_child(label)

# ------------------------------------------------------------------ 재질
func _noise_tex(freq: float, normal := false, lo := Color(0.35, 0.35, 0.35), hi := Color(0.65, 0.65, 0.65), seed_ := 1) -> NoiseTexture2D:
	var n := FastNoiseLite.new()
	n.seed = seed_
	n.noise_type = FastNoiseLite.TYPE_SIMPLEX_SMOOTH
	n.frequency = freq
	n.fractal_octaves = 5
	var t := NoiseTexture2D.new()
	t.width = 1024
	t.height = 1024
	t.seamless = true
	t.noise = n
	if normal:
		t.as_normal_map = true
		t.bump_strength = 6.0
	else:
		var g := Gradient.new()
		g.set_color(0, lo)
		g.set_color(1, hi)
		t.color_ramp = g
	return t

func _mat(cls: String, base: Color, wet: float) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = base
	m.uv1_triplanar = true
	m.uv1_world_triplanar = true
	m.uv1_triplanar_sharpness = 4.0
	m.roughness = 0.85
	m.cull_mode = BaseMaterial3D.CULL_DISABLED     # 원본 메시의 면 방향이 섞여 있음
	match cls:
		"road":
			m.albedo_color = Color(0.24, 0.24, 0.24) * (1.0 - 0.35 * wet)
			m.albedo_texture = _noise_tex(0.08, false, Color(0.55, 0.55, 0.56), Color(1.15, 1.15, 1.15), 3)
			m.normal_enabled = true
			m.normal_texture = _noise_tex(0.25, true, Color.BLACK, Color.WHITE, 4)
			m.normal_scale = 0.6
			m.uv1_scale = Vector3(0.35, 0.35, 0.35)
			m.roughness = lerpf(0.92, 0.12, wet)
			if wet > 0.1:          # 물웅덩이: 거칠기 맵으로 군데군데 매끈하게
				m.roughness_texture = _noise_tex(0.02, false, Color(0.2, 0.2, 0.2), Color(1, 1, 1), 9)
		"lane_white", "lane_yellow":
			m.albedo_texture = _noise_tex(0.3, false, Color(0.8, 0.8, 0.8), Color(1, 1, 1), 5)
			m.uv1_scale = Vector3(0.5, 0.5, 0.5)
			m.roughness = lerpf(0.6, 0.2, wet)
		"barrier", "tunnel":
			m.albedo_texture = _noise_tex(0.15, false, Color(0.8, 0.8, 0.78), Color(1.05, 1.05, 1.0), 6)
			m.uv1_scale = Vector3(0.5, 0.5, 0.5)
			m.roughness = lerpf(0.9, 0.5, wet)
		"guardrail", "pole":
			m.metallic = 0.6
			m.roughness = 0.35
		"terrain":
			m.albedo_color = Color(0.32, 0.42, 0.22)
			m.albedo_texture = _noise_tex(0.04, false, Color(0.6, 0.65, 0.5), Color(1.2, 1.1, 0.9), 7)
			m.uv1_scale = Vector3(0.05, 0.05, 0.05)
			m.roughness = 1.0
		"sign":
			m.emission_enabled = true
			m.emission = base
			m.emission_energy_multiplier = 0.15
	return m

func _load_scene(path: String) -> void:
	var doc := GLTFDocument.new()
	var st := GLTFState.new()
	var err := doc.append_from_file(path, st)
	if err != OK:
		push_error("glb 로드 실패: %s" % path)
		get_tree().quit(1)
		return
	var root := doc.generate_scene(st)
	add_child(root)
	var wet: float = float(info["env"]["wet"])
	for n in root.get_children():
		var nm := String(n.name)
		if nm.begins_with("static_") and n is MeshInstance3D:
			var cls := nm.trim_prefix("static_")
			var mi := n as MeshInstance3D
			var base := Color(0.5, 0.5, 0.5)
			var sm := mi.mesh.surface_get_material(0)
			if sm is BaseMaterial3D:
				base = (sm as BaseMaterial3D).albedo_color
			mi.material_override = _mat(cls, base, wet)
			mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_ON if cls != "terrain" else GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	robot = root.find_child("robot", false, false)
	for i in range(info["protos"].size()):
		var p: Node3D = root.find_child("proto_%d" % i, false, false)
		p.visible = false
		protos.append(p)
	for i in range(info["slots"].size()):
		var v := _make_vehicle(int(info["slots"][i]), info["slot_colors"][i])
		v.visible = false
		slots.append(v)
	for a in info["accident"]:
		var v := _make_vehicle(int(a["proto"]), a["color"], true)
		v.transform = _xf(a["T"])

func _make_vehicle(pi: int, col: Array, hazard := false) -> Node3D:
	var v: Node3D = protos[pi].duplicate()
	v.visible = true
	add_child(v)
	var paint := StandardMaterial3D.new()
	paint.albedo_color = Color(col[0], col[1], col[2])
	paint.metallic = 0.55
	paint.roughness = 0.28
	paint.clearcoat_enabled = true
	paint.clearcoat = 0.8
	paint.cull_mode = BaseMaterial3D.CULL_DISABLED
	(v as MeshInstance3D).material_override = paint
	for c in v.get_children():
		if c is MeshInstance3D:
			var head := String(c.name).begins_with("lamp_head")
			(c as MeshInstance3D).material_override = _lamp_mat(head, hazard)
	var p: Dictionary = info["protos"][pi]
	var sl := SpotLight3D.new()          # 전조등 (차량 로컬 x 전방 = Godot x)
	sl.position = Vector3(float(p["length"]) / 2.0 + 0.2, 0.7, 0)
	sl.rotation_degrees = Vector3(0, -90, 0)
	sl.rotate_object_local(Vector3.RIGHT, deg_to_rad(-4.0))
	sl.spot_angle = 28.0
	sl.spot_range = 60.0
	sl.spot_attenuation = 0.6
	sl.light_energy = 8.0
	sl.light_color = Color(1.0, 0.96, 0.88)
	sl.light_volumetric_fog_energy = 0.5
	sl.visible = false
	v.add_child(sl)
	if not hazard:
		slot_lights.append(sl)
	else:
		acc_lights.append(sl)
	return v

func _lamp_mat(head: bool, hazard: bool) -> StandardMaterial3D:
	var key := ("h" if head else "t") + ("z" if hazard else "")
	if mats.has(key):
		return mats[key]
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.albedo_color = Color(1.0, 0.97, 0.88) if head else Color(1.0, 0.06, 0.03)
	m.emission_enabled = true
	m.emission = m.albedo_color
	if hazard and not head:
		m.albedo_color = Color(1.0, 0.55, 0.05)
		m.emission = m.albedo_color
	mats[key] = m
	return m

# ------------------------------------------------------------------ 환경 (하늘·태양·안개·후처리)
func _build_env(e: Dictionary) -> void:
	var tod: String = e["time_of_day"]
	var wx: String = e["weather"]
	var k: float = float(e["intensity"])
	lamps_on = bool(e["lamps"])
	var sky_mat := ProceduralSkyMaterial.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	sky.sky_material = sky_mat
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.ambient_light_sky_contribution = 0.55
	env.reflected_light_source = Environment.REFLECTION_SOURCE_SKY
	env.tonemap_mode = Environment.TONE_MAPPER_ACES
	env.tonemap_white = 6.0
	env.ssao_enabled = true
	env.ssao_radius = 1.2
	env.ssao_intensity = 1.6
	env.glow_enabled = true
	env.glow_hdr_threshold = 1.0
	env.glow_intensity = 0.7
	env.glow_bloom = 0.05
	sun = DirectionalLight3D.new()
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 250.0
	add_child(sun)
	var overcast := 0.0 if wx == "clear" else clampf(0.4 + 0.6 * k, 0.0, 1.0)
	match tod:
		"day":
			sky_mat.sky_top_color = Color(0.30, 0.50, 0.85).lerp(Color(0.55, 0.57, 0.60), overcast)
			sky_mat.sky_horizon_color = Color(0.70, 0.78, 0.88).lerp(Color(0.68, 0.69, 0.70), overcast)
			sun.rotation_degrees = Vector3(-48, -35, 0)
			sun.light_energy = lerpf(2.2, 0.4, overcast)
			env.tonemap_exposure = 1.0
		"dusk":
			sky_mat.sky_top_color = Color(0.22, 0.25, 0.45).lerp(Color(0.30, 0.30, 0.34), overcast)
			sky_mat.sky_horizon_color = Color(0.98, 0.55, 0.32).lerp(Color(0.55, 0.45, 0.42), overcast)
			sun.rotation_degrees = Vector3(-6, 70, 0)
			sun.light_color = Color(1.0, 0.62, 0.38)
			sun.light_energy = lerpf(1.1, 0.2, overcast)
			env.tonemap_exposure = 1.3
		_:
			sky_mat.sky_top_color = Color(0.005, 0.008, 0.02)
			sky_mat.sky_horizon_color = Color(0.03, 0.035, 0.05)
			sky_mat.sky_energy_multiplier = 0.4
			sun.rotation_degrees = Vector3(-35, 120, 0)     # 달빛
			sun.light_color = Color(0.6, 0.7, 1.0)
			sun.light_energy = 0.04
			sun.shadow_enabled = false
			env.tonemap_exposure = 1.6
			env.glow_intensity = 0.6
			env.glow_bloom = 0.03
	sky_mat.ground_horizon_color = sky_mat.sky_horizon_color
	sky_mat.ground_bottom_color = sky_mat.sky_horizon_color * 0.4
	sky_mat.sun_angle_max = 10.0
	# 대기 산란 / 안개: 가시거리 V → 소광계수 ≈ 3/V
	var vis: float = max(float(e["visibility"]), 30.0)
	env.fog_enabled = true
	env.fog_light_color = sky_mat.sky_horizon_color.lerp(Color(0.6, 0.62, 0.65), 0.3 if tod == "day" else 0.0)
	env.fog_density = 3.0 / vis * (0.5 if wx == "clear" else 1.0)
	env.fog_sky_affect = 0.25 if wx == "clear" else 0.9
	env.fog_aerial_perspective = 0.3
	if wx in ["fog", "rain", "snow"] or tod != "day":
		env.volumetric_fog_enabled = true
		env.volumetric_fog_density = clampf(1.2 / vis, 0.0005, 0.05) if wx != "clear" else 0.0015
		env.volumetric_fog_albedo = Color(0.85, 0.87, 0.9)
		env.volumetric_fog_length = 160.0
		env.volumetric_fog_temporal_reprojection_enabled = true
		env.volumetric_fog_sky_affect = 0.0
	if float(e["wet"]) > 0.1:
		env.ssr_enabled = true
		env.ssr_max_steps = 96
		env.ssr_fade_in = 0.1
		env.ssr_fade_out = 3.0
		env.ssr_depth_tolerance = 0.3
	world_env = WorldEnvironment.new()
	world_env.environment = env
	add_child(world_env)

func _build_lights(d: Dictionary) -> void:
	var on: bool = bool(d["env"]["street_lights"])
	var head_mesh := BoxMesh.new()
	head_mesh.size = Vector3(0.7, 0.12, 0.3)
	var head_mat := StandardMaterial3D.new()
	head_mat.emission_enabled = true
	head_mat.emission = Color(1.0, 0.86, 0.62)
	head_mat.emission_energy_multiplier = 6.0 if on else 0.0
	head_mat.albedo_color = Color(0.8, 0.8, 0.8)
	head_mesh.material = head_mat
	for p in d["street_lights"]:
		var pos := Vector3(p[0], p[1], p[2])
		var mi := MeshInstance3D.new()
		mi.mesh = head_mesh
		mi.position = pos
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		add_child(mi)
		if on:
			var sl := SpotLight3D.new()
			sl.position = pos - Vector3(0, 0.15, 0)
			sl.rotation_degrees = Vector3(-90, 0, 0)
			sl.spot_angle = 62.0
			sl.spot_range = 32.0
			sl.spot_attenuation = 0.7
			sl.light_energy = 12.0
			sl.light_color = Color(1.0, 0.86, 0.62)
			sl.light_volumetric_fog_energy = 2.0
			add_child(sl)
	var tmesh := BoxMesh.new()
	tmesh.size = Vector3(1.2, 0.08, 0.25)
	var tmat := StandardMaterial3D.new()
	tmat.emission_enabled = true
	tmat.emission = Color(1.0, 0.72, 0.4)
	tmat.emission_energy_multiplier = 5.0
	tmesh.material = tmat
	for p in d["tunnel_lights"]:
		var pos := Vector3(p[0], p[1], p[2])
		var mi := MeshInstance3D.new()
		mi.mesh = tmesh
		mi.position = pos
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		add_child(mi)
		var ol := OmniLight3D.new()
		ol.position = pos - Vector3(0, 0.3, 0)
		ol.omni_range = 16.0
		ol.omni_attenuation = 1.2
		ol.light_energy = 2.5
		ol.light_color = Color(1.0, 0.75, 0.45)
		add_child(ol)
	for key in mats:            # 차량 등화 세기
		var m: StandardMaterial3D = mats[key]
		var head: bool = key.begins_with("h")
		m.emission_energy_multiplier = (3.0 if head else 2.5) if lamps_on else (0.6 if head else 0.4)
		if key.ends_with("z"):
			m.emission_energy_multiplier = 6.0
	for sl in slot_lights + acc_lights:
		sl.visible = lamps_on

# ------------------------------------------------------------------ 비 / 눈 파티클 (카메라를 따라다님)
func _build_weather(e: Dictionary) -> void:
	var wx: String = e["weather"]
	if wx != "rain" and wx != "snow":
		return
	var k: float = float(e["intensity"])
	rain = GPUParticles3D.new()
	var pm := ParticleProcessMaterial.new()
	pm.emission_shape = ParticleProcessMaterial.EMISSION_SHAPE_BOX
	pm.emission_box_extents = Vector3(22, 9, 22)      # 높이 방향으로도 넓게 → 시작부터 화면 전체에
	var q := QuadMesh.new()
	var m := StandardMaterial3D.new()
	m.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	m.roughness = 0.2              # 조명을 받아야 보임 (야간엔 가로등·전조등 근처만)
	if wx == "rain":
		rain.amount = int(4000 + 14000 * k)
		rain.lifetime = 1.6
		pm.direction = Vector3(0.08, -1, 0)
		pm.spread = 2.0
		pm.initial_velocity_min = 9.0
		pm.initial_velocity_max = 11.0
		pm.gravity = Vector3(0, -2, 0)
		pm.particle_flag_align_y = true
		q.size = Vector2(0.012, 0.6)
		m.albedo_color = Color(0.8, 0.85, 0.9, 0.35)
		m.billboard_mode = BaseMaterial3D.BILLBOARD_FIXED_Y
	else:
		rain.amount = int(3000 + 9000 * k)
		rain.lifetime = 7.0
		pm.direction = Vector3(0, -1, 0)
		pm.spread = 25.0
		pm.initial_velocity_min = 1.0
		pm.initial_velocity_max = 2.0
		pm.gravity = Vector3(0, -0.6, 0)
		pm.turbulence_enabled = true
		pm.turbulence_noise_strength = 0.6
		q.size = Vector2(0.025, 0.025)
		m.albedo_color = Color(1, 1, 1, 0.9)
		m.billboard_mode = BaseMaterial3D.BILLBOARD_PARTICLES
	q.material = m
	rain.process_material = pm
	rain.draw_pass_1 = q
	rain.local_coords = false
	rain.preprocess = 3.0
	rain.visibility_aabb = AABB(Vector3(-40, -40, -40), Vector3(80, 60, 80))
	add_child(rain)

# ------------------------------------------------------------------ 상태 수신 / 캡처
func _xf(a: Array) -> Transform3D:
	return Transform3D(Basis(Vector3(a[0], a[4], a[8]), Vector3(a[1], a[5], a[9]), Vector3(a[2], a[6], a[10])),
		Vector3(a[3], a[7], a[11]))

func _apply(msg: Dictionary) -> void:
	cam.transform = _xf(msg["cam"])
	if robot:
		robot.transform = _xf(msg["robot"])
	var now := {}
	for row in msg["veh"]:
		var i := int(row[0])
		var v: Node3D = slots[i]
		v.transform = _xf(row.slice(1))
		v.visible = true
		now[i] = true
	for i in shown:
		if not now.has(i):
			slots[i].visible = false
	shown = now
	if rain:
		var target := cam.global_position + Vector3(0, 6, 0) - cam.global_transform.basis.z * 14.0
		var jump := rain.global_position.distance_to(target) > 20.0
		rain.global_position = target
		if jump:                   # 처음 / 순간이동: 그 자리에서 미리 계산된 상태로 다시 시작
			rain.restart()
	label.text = "Godot %s  ·  t = %.1f s  ·  차량 %d  ·  %d fps" % [Engine.get_version_info()["string"], float(msg["t"]), now.size(), Engine.get_frames_per_second()]
	if msg.has("capture"):
		pending_capture = msg["capture"]
		pending_shm = bool(msg.get("raw", false))
		pending_seq = int(msg["seq"])
		wait_frames = int(msg.get("settle", 24))   # 볼류메트릭 안개 시간 누적 · 파티클 안정화

func _process(_dt: float) -> void:
	while udp.get_available_packet_count() > 0:
		var msg = JSON.parse_string(udp.get_packet().get_string_from_utf8())
		if typeof(msg) != TYPE_DICTIONARY:
			continue
		if msg.get("type") == "quit":
			get_tree().quit()
			return
		if msg.get("type") == "state":
			_apply(msg)
	if pending_capture != "":
		wait_frames -= 1
		if wait_frames <= 0:
			var path := pending_capture
			pending_capture = ""
			await RenderingServer.frame_post_draw
			var img := sv.get_texture().get_image()
			if pending_shm:              # 원시 RGB8 (공유 메모리 파일) — 스트리밍용
				img.convert(Image.FORMAT_RGB8)
				var f := FileAccess.open(path, FileAccess.WRITE)
				f.store_buffer(img.get_data())
				f.close()
			else:
				img.save_png(path)
			_send({"type": "captured", "seq": pending_seq, "path": path, "w": img.get_width(), "h": img.get_height(),
				"fps": Engine.get_frames_per_second()})
