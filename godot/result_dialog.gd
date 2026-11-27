extends Control
## 시험 결과 창: PASS/FAIL, 판정 기준 표, 주요 지표, 결과 폴더 · 회귀 시나리오로 저장 · 다시 시험.

signal action(name: String, arg: Variant)

const K = preload("res://ui_kit.gd")

var box: VBoxContainer
var font_bold: Font

func _init() -> void:
	set_anchors_preset(Control.PRESET_FULL_RECT)
	visible = false
	var dim := ColorRect.new()
	dim.color = Color(0, 0, 0, 0.5)
	dim.set_anchors_preset(Control.PRESET_FULL_RECT)
	add_child(dim)
	var cc := CenterContainer.new()
	cc.set_anchors_preset(Control.PRESET_FULL_RECT)
	cc.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(cc)
	var pc := PanelContainer.new()
	pc.add_theme_stylebox_override("panel", K.panel_style(K.BG1, 14, 22, K.BORDER))
	pc.custom_minimum_size = Vector2(680, 0)
	K.eat_mouse(pc)
	cc.add_child(pc)
	box = VBoxContainer.new()
	box.add_theme_constant_override("separation", 10)
	pc.add_child(box)

func show_result(r: Dictionary) -> void:
	for c in box.get_children():
		c.queue_free()
	var ok: bool = r["ok"]
	var h := HBoxContainer.new()
	h.add_theme_constant_override("separation", 12)
	box.add_child(h)
	var big := K.label(K.ic(K.I_CHECK if ok else K.I_XCIRCLE) + "  " + ("PASS" if ok else "FAIL"), K.OK if ok else K.DANGER, 26)
	big.set_meta("bold", true)
	h.add_child(big)
	var sub := K.label("%s · %s · %s s%s" % [r["name"], r["reason"], str(r["sim_time"]),
		"" if r["complete"] else " · 시험 시간 전 종료 (참고용)"], K.MUTED, 14)
	sub.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	sub.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	sub.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(sub)
	# 판정 기준 표
	var tbl := PanelContainer.new()
	tbl.add_theme_stylebox_override("panel", K.panel_style(K.BG0, 8, 12))
	box.add_child(tbl)
	var g := GridContainer.new()
	g.columns = 4
	g.add_theme_constant_override("h_separation", 18)
	g.add_theme_constant_override("v_separation", 6)
	tbl.add_child(g)
	for t in ["기준", "기대", "실제", "판정"]:
		var l := K.label(t, K.MUTED, 13)
		l.set_meta("bold", true)
		g.add_child(l)
	for row in r["rows"]:
		var name := K.label(str(row[0]), K.TEXT, 14)
		name.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		g.add_child(name)
		g.add_child(K.label(str(row[1]), K.TEXT, 14))
		g.add_child(K.label(str(row[2]), K.TEXT, 14))
		var st: String = row[3]
		g.add_child(K.label(st, {"PASS": K.OK, "FAIL": K.DANGER}.get(st, K.MUTED), 14))
	var mt := K.label(str(r["metrics"]), K.MUTED, 13)
	mt.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	box.add_child(mt)
	var dl := K.label("저장: " + str(r["dir"]), K.MUTED, 12)
	dl.autowrap_mode = TextServer.AUTOWRAP_ARBITRARY
	box.add_child(dl)
	var bh := HBoxContainer.new()
	bh.add_theme_constant_override("separation", 6)
	box.add_child(bh)
	var f := K.btn(K.ic(K.I_FOLDER) + "  결과 폴더")
	var d: String = r["dir"]
	f.pressed.connect(func(): action.emit("open", d))
	bh.add_child(f)
	var sv := K.btn(K.ic(K.I_LIST_CHECK) + "  회귀 시나리오로 저장",
		"같은 조건(시드 고정)을 scenarios/ 에 저장 → 회귀 모드에서 반복 실행")
	sv.pressed.connect(func():
		action.emit("save_regression", null)
		sv.disabled = true
		sv.text = K.ic(K.I_CHECK) + "  저장함")
	bh.add_child(sv)
	var sp := Control.new()
	sp.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	bh.add_child(sp)
	var again := K.primary(K.btn(K.ic(K.I_ROTATE) + "  다시 시험"))
	again.pressed.connect(func():
		visible = false
		action.emit("restart", null))
	bh.add_child(again)
	var close := K.btn("닫기")
	close.pressed.connect(func(): visible = false)
	bh.add_child(close)
	if font_bold:
		_bold(box)
	visible = true

func _bold(n: Node) -> void:
	for c in n.get_children():
		if c is Label and c.get_meta("bold", false):
			c.add_theme_font_override("font", font_bold)
		_bold(c)
