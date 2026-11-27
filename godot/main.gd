extends Node
## 진입점: --client 면 BeaconSim 앱(백엔드 TCP 연결), 아니면 카메라 전용 렌더러(기존 ImGui 앱 연동)

func _ready() -> void:
	var args := {}
	for a in OS.get_cmdline_user_args():
		var kv: PackedStringArray = a.trim_prefix("--").split("=", true, 1)
		args[kv[0]] = kv[1] if kv.size() > 1 else ""
	var node: Node3D
	if args.has("client"):
		node = preload("res://client.gd").new()
	else:
		node = preload("res://camera_renderer.gd").new()
	add_child(node)
	node.start(args)
