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
	"tunnel": 7, "vehicle": 8, "robot": 9, "sign": 10, "pole": 11, "accident": 12, "pedestrian": 13,
	"truck": 14, "bus": 15, "trailer": 16}
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
var slot_meta: Array = []      # 슬롯별 {trailer, brake[], sigl[], sigr[], spot, glow, hb}
var acc_meta: Array = []       # 사고 차량 (비상등 깜빡임)
var robot_pos := Vector3.ZERO
var shown := {}
var mats := {}
var lamps_on := false
var beacon_nodes := {}         # 비콘 조명: kind → {light, mesh}
var beacon_mats := {}

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
	slot_meta.clear()
	acc_meta.clear()
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

## 차량: 슬롯 번호 배열 + 변환(12개씩) + 등화 비트(1 브레이크, 2 왼쪽, 4 오른쪽 깜빡이, 8 상향등) + 트레일러 꺾임각
func set_vehicles(idx: PackedInt32Array, T: PackedFloat32Array, lamp := PackedByteArray(),
		art := PackedFloat32Array(), t := 0.0) -> void:
	var now := {}
	var has_l := lamp.size() == idx.size()
	var has_a := art.size() == idx.size()
	for k in range(idx.size()):
		var i := idx[k]
		var v: Node3D = slots[i]
		v.transform = xf_at(T, k)
		v.visible = true
		now[i] = true
		var md: Dictionary = slot_meta[i]
		if has_a and md["trailer"] != null:
			(md["trailer"] as Node3D).basis = Basis(Vector3.UP, art[k])
		var b := int(lamp[k]) if has_l else 0
		_set_lamps(md, b, v.position.distance_to(robot_pos) < 70.0)
	for i in shown:
		if not now.has(i):
			slots[i].visible = false
	shown = now
	var on := fmod(t * 1.5, 1.0) < 0.5           # 사고 차량 비상등
	for md in acc_meta:
		_set_lamps(md, 6 if on else 0, true)

func _set_lamps(md: Dictionary, b: int, near: bool) -> void:
	if b == int(md["b"]) and near == bool(md["near"]):
		return
	md["b"] = b
	md["near"] = near
	for n in md["brake"]:
		n.visible = (b & 1) != 0
	for n in md["sigl"]:
		n.visible = (b & 2) != 0
	for n in md["sigr"]:
		n.visible = (b & 4) != 0
	var sl = md["spot"]
	if sl != null:                               # 상향등: 더 멀리·좁게·덜 숙여서
		var hb := (b & 8) != 0
		(sl as SpotLight3D).spot_range = 110.0 if hb else 55.0
		(sl as SpotLight3D).spot_angle = 20.0 if hb else 30.0
		(sl as SpotLight3D).light_energy = 13.0 if hb else 7.0
		(sl as SpotLight3D).rotation_degrees = Vector3(-1.5 if hb else -4.5, -90, 0)
	var g = md["glow"]
	if g != null:                                # 야간 후미등 붉은 반사 (가까운 차만: 광원 수 제한)
		(g as OmniLight3D).visible = lamps_on and near
		(g as OmniLight3D).light_energy = 0.9 if (b & 1) else 0.25

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
			m.albedo_color = Color(base.r * 1.09, base.g * 1.09, base.b * 1.05) * (1.0 - 0.35 * wet)   # 보수 패치는 색이 다름
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
			var cls := nm.trim_prefix("static_").get_slice("__", 0)     # 같은 클래스·다른 색 파트: static_<클래스>__<번호>
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
		var stl = info["slot_style"][i] if info.has("slot_style") else [0, 0.035]
		var v := _make_vehicle(int(info["slots"][i]), info["slot_colors"][i], false, false, stl)
		v.visible = false
		slots.append(v)
	for a in info["accident"]:
		var v := _make_vehicle(int(a["proto"]), a["color"], true, bool(a.get("ped", false)), a.get("style", [0, 0.035]))
		v.transform = xf(a["T"])

static func set_layers(n: Node, layer: int) -> void:
	if n.get_meta("label", false):
		return
	if n is VisualInstance3D and not (n is Light3D):
		(n as VisualInstance3D).layers = layer
	for c in n.get_children():
		set_layers(c, layer)

func _make_vehicle(pi: int, col: Array, hazard := false, ped := false, style = [0, 0.035]) -> Node3D:
	var v: Node3D = protos[pi].duplicate()
	v.visible = true
	dyn.add_child(v)
	var paint := StandardMaterial3D.new()
	paint.albedo_color = Color(col[0], col[1], col[2])
	if ped:                                  # 사람: 무광 옷감, 등화 없음
		paint.roughness = 0.9
		(v as MeshInstance3D).material_override = paint
		set_layers(v, 16)
		add_label_copies(v, "pedestrian")
		return v
	paint.metallic = 0.55
	paint.roughness = 0.28
	paint.clearcoat_enabled = true
	paint.clearcoat = 0.8
	paint.cull_mode = BaseMaterial3D.CULL_DISABLED
	(v as MeshInstance3D).material_override = paint
	var p: Dictionary = info["protos"][pi]
	var md := {"trailer": null, "brake": [], "sigl": [], "sigr": [], "spot": null, "glow": null, "b": -1, "near": false}
	var tr := v.find_child("trailer", false, false) as Node3D
	md["trailer"] = tr
	var nodes: Array = v.get_children()
	if tr:
		nodes += tr.get_children()
	for c in nodes:
		if not (c is MeshInstance3D):
			continue
		var nm := String(c.name)
		if nm.begins_with("part_"):         # 차종별 부품: 유리(선팅)·타이어·휠(색)·검은 플라스틱·적재함·적재물
			var pm := _part_mat(nm.substr(5), p.get("cargo_color"), style)
			if pm:
				(c as MeshInstance3D).material_override = pm
			continue
		var kind := nm.get_slice("_", 1)         # lamp_<kind>_<j>
		(c as MeshInstance3D).material_override = _lamp_mat(kind)
		if kind in ["brake", "sigl", "sigr"]:
			md[kind].append(c)
			c.visible = false
	var sl := SpotLight3D.new()          # 전조등 (차량 로컬 x 전방 = Godot x), 하향등 기본
	sl.position = Vector3(float(p["length"]) / 2.0 + 0.2, float(p.get("head_z", 0.7)), 0)
	sl.rotation_degrees = Vector3(-4.5, -90, 0)
	sl.spot_angle = 30.0
	sl.spot_range = 55.0
	sl.spot_attenuation = 0.8
	sl.spot_angle_attenuation = 1.6
	sl.light_energy = 7.0
	sl.light_color = Color(1.0, 0.96, 0.88)
	sl.light_volumetric_fog_energy = 0.5
	sl.visible = false
	v.add_child(sl)
	md["spot"] = sl
	var glow := OmniLight3D.new()        # 후미등 붉은 빛 (뒤 노면·앞차 범퍼)
	var rear := -float(p["length"]) / 2.0 - 0.35
	glow.position = Vector3(rear, float(p.get("tail_z", 0.8)), 0)
	glow.omni_range = 3.0
	glow.omni_attenuation = 1.5
	glow.light_color = Color(1.0, 0.08, 0.04)
	glow.light_energy = 0.25
	glow.light_volumetric_fog_energy = 0.0
	glow.shadow_enabled = false
	glow.visible = false
	v.add_child(glow)
	md["glow"] = glow
	if not hazard:
		slot_lights.append(sl)
		slot_meta.append(md)
	else:
		acc_lights.append(sl)
		acc_meta.append(md)
	set_layers(v, 16)
	var kind := str(p.get("kind", "car"))
	add_label_copies(v, "accident" if hazard else ("truck" if kind == "truck" else "bus" if kind == "bus" else "vehicle"))
	if tr and not hazard:                    # 트레일러: 카메라 정답 라벨 trailer (트랙터는 truck)
		_relabel(tr, "trailer")
	return v

## 이미 붙인 라벨 복제본의 라벨 재질만 바꿈 (트레일러 = trailer)
func _relabel(n: Node, cls: String) -> void:
	for c in n.get_children():
		if c.get_meta("label", false) and c is MeshInstance3D:
			(c as MeshInstance3D).material_override = label_mat(cls)
		_relabel(c, cls)

## 차량 부품 재질 (모든 차량이 공유). 적재함(cargo)은 차종별 색
func _part_mat(tag: String, cargo = null, style = [0, 0.035]) -> StandardMaterial3D:
	var key := "part_" + tag
	if tag == "cargo":
		if typeof(cargo) != TYPE_ARRAY:
			return null
		key += "_%.2f_%.2f_%.2f" % [cargo[0], cargo[1], cargo[2]]
	elif tag == "glass":
		key += "_%.3f" % float(style[1])
	elif tag == "rim":
		key += "_%d" % int(style[0])
	if mats.has(key):
		return mats[key]
	var m := StandardMaterial3D.new()
	m.cull_mode = BaseMaterial3D.CULL_DISABLED
	match tag:
		"glass":                         # 선팅 유리 (차마다 농도 다름): 어두운 색 + 강한 반사
			var tb := float(style[1])
			m.albedo_color = Color(tb, tb * 1.15, tb * 1.4)
			m.metallic = 0.2
			m.roughness = 0.05
			m.metallic_specular = 0.9
		"tire":
			m.albedo_color = Color(0.035, 0.035, 0.035)
			m.roughness = 0.92
		"rim":
			var rc: Array = info.get("rims", [[0.62, 0.63, 0.65]])[clampi(int(style[0]), 0, info.get("rims", [[0]]).size() - 1)]
			m.albedo_color = Color(rc[0], rc[1], rc[2])
			m.metallic = 0.8
			m.roughness = 0.3
		"cargo":                         # 적재함 (탑차·윙 알루미늄, 덤프 철판, 탱크)
			m.albedo_color = Color(cargo[0], cargo[1], cargo[2])
			m.metallic = 0.35
			m.roughness = 0.45
		"load":                          # 적재물 (방수포)
			m.albedo_color = Color(0.16, 0.30, 0.55)
			m.roughness = 0.85
		_:                               # trim: 범퍼·그릴·미러 등 검은 플라스틱
			m.albedo_color = Color(0.06, 0.06, 0.065)
			m.roughness = 0.6
	mats[key] = m
	return m

## 등화 재질: head 전조등, tail 후미등, brake 브레이크등, sigl/sigr 방향지시등 (밝기는 _build_lights 에서)
func _lamp_mat(kind: String) -> StandardMaterial3D:
	if kind.begins_with("sig"):
		kind = "sig"
	var key := "lamp_" + kind
	if mats.has(key):
		return mats[key]
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.albedo_color = {"head": Color(1.0, 0.97, 0.88), "tail": Color(1.0, 0.06, 0.03), "brake": Color(1.0, 0.03, 0.02),
		"sig": Color(1.0, 0.55, 0.05)}.get(kind, Color(1, 1, 1))
	m.emission_enabled = true
	m.emission = m.albedo_color
	m.emission_energy_multiplier = _lamp_energy(kind)
	mats[key] = m
	return m

func _lamp_energy(kind: String) -> float:
	match kind:
		"head":
			return 3.0 if lamps_on else 0.6
		"tail":
			return 2.5 if lamps_on else 0.4
		"brake":
			return 6.0
		_:
			return 7.0

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
			# 야간 조도: 달빛(0~1) · 하늘 밝음(도시 광공해, 0~1) — 값은 백엔드에서 랜덤 또는 지정
			var moon: float = float(e.get("moon", 0.5)) * (1.0 - overcast * 0.8)
			var glow: float = float(e.get("sky_glow", 0.2))
			sky_mat.sky_top_color = Color(0.003, 0.005, 0.014).lerp(Color(0.03, 0.035, 0.06), moon * 0.6)
			sky_mat.sky_horizon_color = Color(0.015, 0.018, 0.03).lerp(Color(0.16, 0.11, 0.07), glow)
			sky_mat.sky_energy_multiplier = 0.25 + 0.5 * glow + 0.25 * moon
			sun.rotation_degrees = Vector3(-35, 120, 0)     # 달빛
			sun.light_color = Color(0.6, 0.7, 1.0)
			sun.light_energy = 0.006 + 0.07 * moon
			sun.shadow_enabled = moon > 0.6
			env.ambient_light_sky_contribution = 0.4 + 0.4 * glow
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
	street_head_mat.emission_energy_multiplier = 0.0      # 등기구는 기본 꺼짐, 켜진 램프만 빛나는 덮개를 씌움
	var on_mask: Array = d.get("street_on", [])
	var glow_mesh := BoxMesh.new()
	glow_mesh.size = Vector3(0.72, 0.13, 0.32)
	var glow_mat := StandardMaterial3D.new()
	glow_mat.emission_enabled = true
	glow_mat.emission = Color(1.0, 0.86, 0.62)
	glow_mat.emission_energy_multiplier = 6.0
	glow_mat.albedo_color = Color(1.0, 0.9, 0.7)
	glow_mesh.material = glow_mat
	var street: Array = d["street_lights"]
	for i in range(street.size()):
		if i < on_mask.size() and bool(on_mask[i]):
			var p = street[i]
			var gm := MeshInstance3D.new()
			gm.mesh = glow_mesh
			gm.position = Vector3(p[0], p[1], p[2])
			gm.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
			dyn.add_child(gm)
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
	for key in mats:            # 차량 등화 세기 (낮/밤)
		if String(key).begins_with("lamp_"):
			(mats[key] as StandardMaterial3D).emission_energy_multiplier = _lamp_energy(String(key).substr(5))
	for sl in slot_lights + acc_lights:
		sl.visible = lamps_on
	_build_beacon_lights(d.get("beacon_lights", []))

## 비콘 조명 (서치라이트·경광등): 센서타워 링크에 붙여 함께 접히고 펴진다. 켜짐은 set_beacon_lights 로
func _build_beacon_lights(list: Array) -> void:
	for k in beacon_nodes:                    # 비콘은 정적 장면(재사용)에 있으므로 예전 조명을 지운다
		for n in beacon_nodes[k]["nodes"]:
			if is_instance_valid(n):
				n.get_parent().remove_child(n)
				n.queue_free()
	beacon_nodes = {}
	if robot == null:
		return
	for b in list:
		var kind: String = b["kind"]
		var parent: Node3D = robot
		if b["link"] != "base":
			var ln := robot.find_child("link_" + str(b["link"]), false, false) as Node3D
			if ln:
				parent = ln
		var pos := Vector3(b["pos"][0], b["pos"][1], b["pos"][2])
		var dir := Vector3(b["dir"][0], b["dir"][1], b["dir"][2]).normalized()
		var nodes := []
		var light: Light3D
		if kind == "search":
			var sl := SpotLight3D.new()
			sl.transform = Transform3D(Basis.looking_at(dir, Vector3.UP), pos + dir * 0.02)
			sl.spot_angle = float(b["beam_deg"]) / 2.0
			sl.spot_range = float(b["range_m"])
			sl.spot_attenuation = 1.0                                      # 거리 제곱에 반비례
			sl.spot_angle_attenuation = 2.5                                # 가장자리 부드럽게
			sl.light_energy = float(b["power_lm"]) / 1000.0 * float(b.get("energy_per_klm", 6.0))
			sl.light_color = Color(b["color"][0], b["color"][1], b["color"][2])
			sl.light_volumetric_fog_energy = 0.15    # 안개 속 빛줄기 (크면 낮은 해상도 안개에서 번짐)
			sl.shadow_enabled = true
			light = sl
			var spill := SpotLight3D.new()                # 주변으로 퍼지는 약한 빛 (실제 LED 등의 넓은 산란광)
			spill.transform = sl.transform
			spill.spot_angle = minf(float(b["beam_deg"]) * 1.6, 75.0)
			spill.spot_range = float(b["range_m"]) * 0.4
			spill.spot_attenuation = 1.0
			spill.spot_angle_attenuation = 1.5
			spill.light_energy = sl.light_energy * 0.08
			spill.light_color = sl.light_color
			spill.light_volumetric_fog_energy = 0.0
			spill.light_cull_mask = 0xFFFFF & ~32
			spill.visible = false
			parent.add_child(spill)
			nodes.append(spill)
			var lens := MeshInstance3D.new()          # 등기구 앞면 (켜지면 빛남)
			var bm := BoxMesh.new()
			bm.size = Vector3(0.008, 0.02, 0.09)
			lens.mesh = bm
			lens.transform = Transform3D(Basis.looking_at(dir, Vector3.UP), pos)
			lens.layers = 32
			parent.add_child(lens)
			nodes.append(lens)
			beacon_mats[kind] = _beacon_mat(Color(1.0, 0.97, 0.9))
			lens.material_override = beacon_mats[kind]
		else:
			var col := Color(1.0, 0.08, 0.05) if kind == "red" else Color(0.1, 0.3, 1.0)
			var ol := SpotLight3D.new()               # 경광등: 전방으로 넓게 쏘는 빛 (노면·차량·가드레일을 물들임)
			ol.transform = Transform3D(Basis.looking_at(dir, Vector3.UP), pos + dir * 0.02)
			ol.spot_angle = float(b.get("beam_deg", 70.0)) / 2.0
			ol.spot_range = float(b["range_m"])
			ol.spot_attenuation = 1.0
			ol.spot_angle_attenuation = 1.5
			ol.light_energy = float(b["energy"])
			ol.light_color = col
			ol.light_volumetric_fog_energy = 0.0     # 안개 산란 끔 (낮은 해상도 안개에서 멀리까지 붉게 번졌음)
			ol.light_specular = 0.3
			light = ol
			# 앞면 발광창만 빛난다 (하우징 전체가 아니라 LED WINDOW 전면 크기 = robot.yaml size [폭, 높이])
			var win := MeshInstance3D.new()
			var wm := BoxMesh.new()
			wm.size = Vector3(float(b["size"][0]), float(b["size"][1]), 0.003)
			win.mesh = wm
			win.transform = Transform3D(Basis.looking_at(dir, Vector3.UP), pos + dir * 0.002)
			win.layers = 32
			win.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
			parent.add_child(win)
			nodes.append(win)
			beacon_mats[kind] = _beacon_mat(col)
			win.material_override = beacon_mats[kind]
		light.visible = false
		light.light_cull_mask = 0xFFFFF & ~32        # 비콘 자신(레이어 32)은 비추지 않음 (바로 옆이라 하얗게 타 버림)
		parent.add_child(light)
		nodes.append(light)
		beacon_nodes[kind] = {"nodes": nodes, "light": light, "extra": nodes.filter(func(n): return n is Light3D)}
	# 밤·황혼: 비콘 몸체가 새까맣게 묻히지 않게 은은한 중성 보조광 (비콘 레이어만)
	if str(info["env"]["time_of_day"]) != "day":
		var amb := OmniLight3D.new()
		amb.position = Vector3(0.0, 1.6, 0.0)
		amb.omni_range = 4.0
		amb.omni_attenuation = 0.5
		amb.light_energy = 0.25
		amb.light_color = Color(0.75, 0.8, 1.0)
		amb.light_cull_mask = 32
		amb.shadow_enabled = false
		robot.add_child(amb)
		beacon_nodes["fill"] = {"nodes": [amb], "light": amb, "extra": []}

func _beacon_mat(col: Color) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = col * 0.6
	m.emission_enabled = true
	m.emission = col
	m.emission_energy_multiplier = 0.0
	return m

## 조명 켜짐 {red, blue, search} (상태 메시지마다)
func set_beacon_lights(st) -> void:
	if typeof(st) != TYPE_DICTIONARY:
		return
	for kind in beacon_nodes:
		if kind == "fill":
			continue
		var on := bool(st.get(kind, false))
		beacon_nodes[kind]["light"].visible = on
		for x in beacon_nodes[kind]["extra"]:
			x.visible = on
		if beacon_mats.has(kind):
			(beacon_mats[kind] as StandardMaterial3D).emission_energy_multiplier = (20.0 if kind == "search" else 6.0) if on else 0.0

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

