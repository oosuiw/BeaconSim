extends Node3D

## 정적 장면(glb) + 재질 + 하늘·안개·후처리 + 가로등·터널등 + 차량·비콘. 카메라 전용 렌더러와 Godot 앱이 함께 쓴다.
## 두 부분: 정적(static_root: 도로·지형·터널·비콘·등기구 모양·터널등) / 동적(dyn: 차량·하늘·태양·안개·가로등 빛·비/눈).
## load(): 맵이 같으면(static_id) 정적 부분을 그대로 두고 동적 부분만 다시 만든다 → 재시작이 빠르다.

var info: Dictionary
var static_id := ""
var static_root: Node3D
var dyn: Node3D
var static_meshes: Array = []   # [MeshInstance3D, 분류, 기본색]
var street_head_mat: StandardMaterial3D
static var noise_cache := {}
## 카메라 정답 라벨: 모든 물체의 복제본을 레이어 7(64)에 라벨 번호 색으로 (조명·안개 없음). 정답 카메라만 이 레이어를 본다.
const LABEL_LAYER := 64
const LABEL_STEP := 20.0                 # 라벨 번호 → 색 값 (R = 번호 x 20 / 255), 읽을 때 반올림
static var label_ids := {"road": 1, "lane_white": 2, "lane_yellow": 3, "guardrail": 4, "barrier": 5, "terrain": 6,
	"tunnel": 7, "vehicle": 8, "robot": 9, "sign": 10, "pole": 11, "accident": 12}
static var label_mats := {}

static func label_mat(cls: String) -> StandardMaterial3D:
	var base := cls
	var i := cls.find("_faded")            # 희미한 차선도 차선 (라이다 정답과 같음)
	if i > 0:
		base = cls.substr(0, i)
	var id := int(label_ids.get(base, 0))
	if not label_mats.has(id):
		var m := StandardMaterial3D.new()
		m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		m.albedo_color = Color8(int(id * LABEL_STEP), 0, 0)
		m.cull_mode = BaseMaterial3D.CULL_DISABLED
		m.disable_fog = true
		m.disable_receive_shadows = true
		label_mats[id] = m
	return label_mats[id]

## 메시 인스턴스마다 라벨 복제본 (같은 메시, 자식이라 움직임을 그대로 따라감)
static func add_label_copies(n: Node, cls: String) -> void:
	for c in n.get_children():
		add_label_copies(c, cls)
	if n is MeshInstance3D and not n.get_meta("label", false):
		var mi := n as MeshInstance3D
		var d := MeshInstance3D.new()
		d.mesh = mi.mesh
		d.material_override = label_mat(cls)
		d.layers = LABEL_LAYER
		d.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		d.set_meta("label", true)
		mi.add_child(d)
var world_env: WorldEnvironment
var sun: DirectionalLight3D
var robot: Node3D
var protos: Array = []
var slots: Array = []          # 슬롯별 차량 인스턴스
var slot_lights: Array = []
var acc_lights: Array = []
var shown := {}
var mats := {}
var lamps_on := false

static func xf(a) -> Transform3D:
	return Transform3D(Basis(Vector3(a[0], a[4], a[8]), Vector3(a[1], a[5], a[9]), Vector3(a[2], a[6], a[10])),
		Vector3(a[3], a[7], a[11]))

static func xf_at(f: PackedFloat32Array, k: int) -> Transform3D:
	var o := k * 12
	return Transform3D(Basis(Vector3(f[o], f[o + 4], f[o + 8]), Vector3(f[o + 1], f[o + 5], f[o + 9]),
		Vector3(f[o + 2], f[o + 6], f[o + 10])), Vector3(f[o + 3], f[o + 7], f[o + 11]))

static var perf := OS.get_environment("BEACONSIM_PERF") != ""

static func tlog(what: String, t0: int) -> int:
	var t := Time.get_ticks_msec()
	if perf:
		print("WORLD %s %d ms" % [what, t - t0])
	return t

func build(scene_json: String) -> void:
	load_scene(scene_json)

## 장면 불러오기. 반환: 정적 부분을 재사용했으면 true
func load_scene(scene_json: String) -> bool:
	var t := Time.get_ticks_msec()
	info = JSON.parse_string(FileAccess.get_file_as_string(scene_json))
	var sid: String = str(info.get("static_id", ""))
	var reuse := static_root != null and sid != "" and sid == static_id
	if not reuse:
		if static_root:
			remove_child(static_root)
			static_root.queue_free()
		static_root = Node3D.new()
		add_child(static_root)
		static_meshes.clear()
		_load_static(str(info["static_glb"]))
		_build_static_lights(info)
		static_id = sid
		t = tlog("정적 장면(glb)", t)
	if dyn:
		remove_child(dyn)
		dyn.queue_free()
	dyn = Node3D.new()
	add_child(dyn)
	protos.clear()
	slots.clear()
	slot_lights.clear()
	acc_lights.clear()
	shown = {}
	var wet: float = float(info["env"]["wet"])
	for sm in static_meshes:                 # 노면 젖음 등 환경에 따라 재질만 바꿈
		(sm[0] as MeshInstance3D).material_override = _mat(sm[1], sm[2], wet)
	t = tlog("재질", t)
	_load_vehicles(str(info["vehicles_glb"]))
	t = tlog("차량", t)
	_build_env(info["env"])
	_build_lights(info)
	tlog("환경·조명", t)
	return reuse

## 차량: 슬롯 번호 배열 + 변환(12개씩)
func set_vehicles(idx: PackedInt32Array, T: PackedFloat32Array) -> void:
	var now := {}
	for k in range(idx.size()):
		var i := idx[k]
		var v: Node3D = slots[i]
		v.transform = xf_at(T, k)
		v.visible = true
		now[i] = true
	for i in shown:
		if not now.has(i):
			slots[i].visible = false
	shown = now

func set_robot(T, links = null) -> void:
	if robot:
		robot.transform = xf(T)
		if typeof(links) == TYPE_DICTIONARY:          # 접이식 센서타워·링크 (비콘 기준 로컬 변환)
			for name in links:
				var n := robot.find_child("link_" + str(name), false, false) as Node3D
				if n:
					n.transform = xf(links[name])

## 비·눈 파티클을 카메라 앞에 둔다 (처음/순간이동이면 미리 계산된 상태로 다시 시작)
static func follow_weather(p: GPUParticles3D, cam: Camera3D) -> void:
	if p == null:
		return
	var target := cam.global_position + Vector3(0, 6, 0) - cam.global_transform.basis.z * 14.0
	var jump := p.global_position.distance_to(target) > 20.0
	p.global_position = target
	if jump:
		p.restart()

# ------------------------------------------------------------------ 재질
func _noise_tex(freq: float, normal := false, lo := Color(0.35, 0.35, 0.35), hi := Color(0.65, 0.65, 0.65), seed_ := 1) -> NoiseTexture2D:
	var key := "%s|%s|%s|%s|%s" % [freq, normal, lo, hi, seed_]
	if noise_cache.has(key):                 # 한 번 만든 노이즈 텍스처는 재사용 (재시작 때 다시 만들지 않음)
		return noise_cache[key]
	var t := _make_noise(freq, normal, lo, hi, seed_)
	noise_cache[key] = t
	return t

func _make_noise(freq: float, normal: bool, lo: Color, hi: Color, seed_: int) -> NoiseTexture2D:
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

static func _gltf(path: String) -> Node:
	var doc := GLTFDocument.new()
	var st := GLTFState.new()
	var err := doc.append_from_file(path, st)
	if err != OK:
		push_error("glb 로드 실패: %s" % path)
		return null
	return doc.generate_scene(st)

func _load_static(path: String) -> void:
	var root := _gltf(path)
	if root == null:
		return
	static_root.add_child(root)
	for n in root.get_children():
		var nm := String(n.name)
		if nm.begins_with("static_") and n is MeshInstance3D:
			var cls := nm.trim_prefix("static_")
			var mi := n as MeshInstance3D
			var base := Color(0.5, 0.5, 0.5)
			var sm := mi.mesh.surface_get_material(0)
			if sm is BaseMaterial3D:
				base = (sm as BaseMaterial3D).albedo_color
			static_meshes.append([mi, cls, base])
			add_label_copies(mi, cls)
			mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_ON if cls != "terrain" else GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
			if cls == "tunnel":                   # 레이어: 1 도로 등, 2 터널, 4 지형, 5 차량, 6 비콘 (메인 뷰에서 켜고 끔)
				mi.layers = 2
			elif cls == "terrain":
				mi.layers = 8
	robot = root.find_child("robot", false, false)
	if robot:
		set_layers(robot, 32)
		add_label_copies(robot, "robot")

func _load_vehicles(path: String) -> void:
	var root := _gltf(path)
	if root == null:
		return
	root.visible = false
	dyn.add_child(root)
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
		v.transform = xf(a["T"])

static func set_layers(n: Node, layer: int) -> void:
	if n.get_meta("label", false):
		return
	if n is VisualInstance3D and not (n is Light3D):
		(n as VisualInstance3D).layers = layer
	for c in n.get_children():
		set_layers(c, layer)

func _make_vehicle(pi: int, col: Array, hazard := false) -> Node3D:
	var v: Node3D = protos[pi].duplicate()
	v.visible = true
	dyn.add_child(v)
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
	set_layers(v, 16)
	add_label_copies(v, "accident" if hazard else "vehicle")
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
	dyn.add_child(sun)
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
	dyn.add_child(world_env)

## 정적: 가로등 등기구 모양(밤에만 빛남 → 재질은 동적에서 조절), 터널등(항상 켜짐)
func _build_static_lights(d: Dictionary) -> void:
	var head_mesh := BoxMesh.new()
	head_mesh.size = Vector3(0.7, 0.12, 0.3)
	street_head_mat = StandardMaterial3D.new()
	street_head_mat.emission_enabled = true
	street_head_mat.emission = Color(1.0, 0.86, 0.62)
	street_head_mat.albedo_color = Color(0.8, 0.8, 0.8)
	head_mesh.material = street_head_mat
	for p in d["street_lights"]:
		var mi := MeshInstance3D.new()
		mi.mesh = head_mesh
		mi.position = Vector3(p[0], p[1], p[2])
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		static_root.add_child(mi)
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
		static_root.add_child(mi)
		var ol := OmniLight3D.new()
		ol.position = pos - Vector3(0, 0.3, 0)
		ol.omni_range = 16.0
		ol.omni_attenuation = 1.2
		ol.light_energy = 2.5
		ol.light_color = Color(1.0, 0.75, 0.45)
		static_root.add_child(ol)

## 동적: 가로등 빛(야간), 차량 등화 세기
func _build_lights(d: Dictionary) -> void:
	var on: bool = bool(d["env"]["street_lights"])
	street_head_mat.emission_energy_multiplier = 6.0 if on else 0.0
	if on:
		for p in d["street_lights"]:
			var sl := SpotLight3D.new()
			sl.position = Vector3(p[0], p[1], p[2]) - Vector3(0, 0.15, 0)
			sl.rotation_degrees = Vector3(-90, 0, 0)
			sl.spot_angle = 62.0
			sl.spot_range = 32.0
			sl.spot_attenuation = 0.7
			sl.light_energy = 12.0
			sl.light_color = Color(1.0, 0.86, 0.62)
			sl.light_volumetric_fog_energy = 2.0
			dyn.add_child(sl)
	for key in mats:            # 차량 등화 세기
		var m: StandardMaterial3D = mats[key]
		var head: bool = key.begins_with("h")
		m.emission_energy_multiplier = (3.0 if head else 2.5) if lamps_on else (0.6 if head else 0.4)
		if key.ends_with("z"):
			m.emission_energy_multiplier = 6.0
	for sl in slot_lights + acc_lights:
		sl.visible = lamps_on

# ------------------------------------------------------------------ 비 / 눈 파티클 (카메라를 따라다님)
func make_weather() -> GPUParticles3D:
	var e: Dictionary = info["env"]
	var wx: String = e["weather"]
	if wx != "rain" and wx != "snow":
		return null
	var k: float = float(e["intensity"])
	var rain := GPUParticles3D.new()
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
	dyn.add_child(rain)
	return rain

