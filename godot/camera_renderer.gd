extends Node3D
const BeaconWorld = preload("res://world.gd")
## 카메라 전용 렌더러 (기존 ImGui 앱의 'Godot 고품질' 카메라). beacon_sim/godot_link.py 가 UDP JSON 으로 상태를 보낸다.
## 실행: godot --path godot -- --scene=<scene.json> --port=P --reply=R [--hidden]

var world: BeaconWorld
var udp := PacketPeerUDP.new()
var reply := PacketPeerUDP.new()
var sv: SubViewport
var cam: Camera3D
var rain: GPUParticles3D
var pending_capture := ""
var pending_shm := false
var pending_seq := 0
var wait_frames := 0
var label: Label

func start(args: Dictionary) -> void:
	var port := int(args.get("port", "47810"))
	var reply_port := int(args.get("reply", str(port + 1)))
	var hidden := args.has("hidden")
	if hidden:   # 최소화하면 렌더링이 멈추고 화면 밖 좌표는 창 관리자가 무시 → 1x1 투명 창, 클릭 통과
		get_viewport().transparent_bg = true
		DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_BORDERLESS, true)
		DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_TRANSPARENT, true)
		DisplayServer.window_set_flag(DisplayServer.WINDOW_FLAG_NO_FOCUS, true)
		DisplayServer.window_set_mouse_passthrough(PackedVector2Array([Vector2(0, 0), Vector2(0, 0), Vector2(0, 0)]))
		DisplayServer.window_set_size(Vector2i(1, 1))
		DisplayServer.window_set_title("BeaconSim 카메라 렌더러")
		Engine.max_fps = 60
	world = BeaconWorld.new()
	add_child(world)
	world.build(args.get("scene", ""))
	_build_viewport(hidden)
	rain = world.make_weather()
	udp.bind(port, "127.0.0.1")
	reply.set_dest_address("127.0.0.1", reply_port)
	await get_tree().create_timer(1.0).timeout      # 노이즈 텍스처(비동기 생성) 준비 대기
	_send({"type": "ready"})

func _send(d: Dictionary) -> void:
	reply.put_packet(JSON.stringify(d).to_utf8_buffer())

func _build_viewport(hidden: bool) -> void:
	var c: Dictionary = world.info["camera"]
	sv = SubViewport.new()
	sv.size = Vector2i(int(c["width"]), int(c["height"]))
	sv.msaa_3d = Viewport.MSAA_4X
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
	var layer := CanvasLayer.new()
	layer.visible = not hidden
	add_child(layer)
	var tr := TextureRect.new()
	tr.texture = sv.get_texture()
	tr.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	tr.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	tr.set_anchors_preset(Control.PRESET_FULL_RECT)
	layer.add_child(tr)
	label = Label.new()
	label.position = Vector2(12, 8)
	layer.add_child(label)

func _apply(msg: Dictionary) -> void:
	cam.transform = BeaconWorld.xf(msg["cam"])
	world.set_robot(msg["robot"], msg.get("links"))
	var idx := PackedInt32Array()
	var T := PackedFloat32Array()
	for row in msg["veh"]:
		idx.append(int(row[0]))
		for k in range(1, 13):
			T.append(float(row[k]))
	var lamp := PackedByteArray()
	for x in msg.get("lamp", []):
		lamp.append(int(x))
	world.robot_pos = BeaconWorld.xf(msg["robot"]).origin
	world.set_vehicles(idx, T, lamp, PackedFloat32Array(msg.get("art", [])), float(msg["t"]))
	BeaconWorld.follow_weather(rain, cam)
	label.text = "t = %.1f s  ·  차량 %d  ·  %d fps" % [float(msg["t"]), idx.size(), Engine.get_frames_per_second()]
	if msg.has("capture"):
		pending_capture = msg["capture"]
		pending_shm = bool(msg.get("raw", false))
		pending_seq = int(msg["seq"])
		wait_frames = int(msg.get("settle", 24))   # 볼류메트릭 안개 누적 · 파티클 안정화

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
