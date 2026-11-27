extends Control
## 시작 화면: 모드(개발/시험/회귀) 선택 · 실행 조건 편집 · 요약 · 시작 / 회귀 시나리오 목록·실행 / 첫 실행 STL 선택.
## 상태는 백엔드의 mode 메시지(beacon_sim/ui_schema.mode_state), 조건 탭은 ui 메시지의 start_tabs 로 그린다.

signal edited(k: String, v: Variant)
signal action(name: String, arg: Variant)

const K = preload("res://ui_kit.gd")
const Form = preload("res://form.gd")
const MODES := [
	["dev", K.I_WRENCH, "개발", "기능 검증", "센서 · TF · 상위 제어기 연동 디버깅. 실행 중에도 조건을 바꿔 바로 적용할 수 있습니다.",
		["교통 끔 (기본)", "실행 중 변경", "평가 없음"]],
	["test", K.I_CLIPBOARD, "시험", "시나리오 평가", "조건 1세트로 시험하고 PASS/FAIL 리포트를 남깁니다. 시험 중 조건은 잠기고, 바꾸면 재시작합니다.",
		["교통 켬 (기본)", "조건 잠금", "리포트 · YAML"]],
	["regress", K.I_LIST_CHECK, "회귀", "일괄 실행", "저장된 시나리오 YAML 을 차례로 자동 실행해 기준 통과 여부를 판정합니다.",
		["화면 없이 고속", "ROS 연동 선택", "report.md"]],
]

var form: Form
var font_bold: Font
var m := {}                      # 최근 mode 메시지
var content: VBoxContainer
var back_btn: Button
var cards := {}
var left_cond: VBoxContainer
var left_reg: VBoxContainer
var right_cond: VBoxContainer
var right_reg: VBoxContainer
var preset_ob: OptionButton
var cond_title: Label
var summary_box: VBoxContainer
var ros_box: VBoxContainer
var start_btn: Button
var last_box: VBoxContainer
var reg_list: VBoxContainer
var reg_title: Label
var reg_ros: CheckBox
var reg_run: Button
var reg_prog: VBoxContainer
var robot_layer: Control
var robot_btn: Button
var robot_log: Label
var yaml_dialog: FileDialog
var stl_dialog: FileDialog
var sigs := {}

func _init() -> void:
	set_anchors_preset(Control.PRESET_FULL_RECT)
	K.eat_mouse(self)
	var bg := ColorRect.new()
	bg.color = Color(K.BG0, 1.0)
	bg.set_anchors_preset(Control.PRESET_FULL_RECT)
	bg.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(bg)
	content = VBoxContainer.new()
	content.add_theme_constant_override("separation", 14)
	add_child(content)
	# 머리글
	var h := HBoxContainer.new()
	h.add_theme_constant_override("separation", 10)
	content.add_child(h)
	h.add_child(K.label(K.ic(K.I_TOWER), K.ACCENT, 30))
	var t := K.label("BeaconSim", K.TEXT, 30)
	t.set_meta("bold", true)
	h.add_child(t)
	var sub := K.label("", K.MUTED, 14)
	sub.name = "sub"
	sub.size_flags_vertical = Control.SIZE_SHRINK_END
	sub.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(sub)
	back_btn = K.btn("")
	back_btn.pressed.connect(func(): action.emit("back", null))
	h.add_child(back_btn)
	# 모드 카드
	var cr := HBoxContainer.new()
	cr.add_theme_constant_override("separation", 14)
	content.add_child(cr)
	for md in MODES:
		cards[md[0]] = _mode_card(cr, md)
	# 본문
	var body := HBoxContainer.new()
	body.add_theme_constant_override("separation", 14)
	body.size_flags_vertical = Control.SIZE_EXPAND_FILL
	content.add_child(body)
	left_cond = _col(body, 1.8)
	left_reg = _col(body, 1.8)
	right_cond = _col(body, 1.0)
	right_reg = _col(body, 1.0)
	_build_cond_left()
	_build_cond_right()
	_build_reg_left()
	_build_reg_right()
	_build_robot_modal()

func _col(parent: Control, ratio: float) -> VBoxContainer:
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 10)
	v.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	v.size_flags_stretch_ratio = ratio
	parent.add_child(v)
	return v

func set_version(v: String) -> void:
	(content.find_child("sub", true, false) as Label).text = \
		"v%s  ·  세이프티 비콘 시뮬레이터  ·  모드를 고르고 조건을 정한 뒤 시작하세요" % v

func set_fonts(bold: Font) -> void:
	font_bold = bold
	form.font_bold = bold
	form.apply_bold(self)

func _process(_dt: float) -> void:
	if not visible:
		return
	var cw := minf(1320.0, size.x - 48.0)
	content.position = Vector2((size.x - cw) / 2.0, 26)
	content.size = Vector2(cw, size.y - 50)

# ------------------------------------------------------------------ 모드 카드
func _mode_card(parent: Control, md: Array) -> PanelContainer:
	var pc := PanelContainer.new()
	pc.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	pc.custom_minimum_size = Vector2(0, 150)
	pc.mouse_default_cursor_shape = Control.CURSOR_POINTING_HAND
	var key: String = md[0]
	pc.gui_input.connect(func(e):
		if e is InputEventMouseButton and e.pressed and e.button_index == MOUSE_BUTTON_LEFT:
			action.emit("select_mode", key))
	parent.add_child(pc)
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 6)
	v.mouse_filter = Control.MOUSE_FILTER_IGNORE
	pc.add_child(v)
	var h := HBoxContainer.new()
	h.mouse_filter = Control.MOUSE_FILTER_IGNORE
	v.add_child(h)
	var title := K.label(K.ic(md[1]) + "  " + md[2], K.MUTED, 22)
	title.set_meta("bold", true)
	title.name = "title"
	h.add_child(title)
	var s := K.label(md[3], K.MUTED, 13)
	s.size_flags_vertical = Control.SIZE_SHRINK_END
	h.add_child(s)
	var d := K.label(md[4], K.TEXT, 14)
	d.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	d.size_flags_vertical = Control.SIZE_EXPAND_FILL
	v.add_child(d)
	var ch := HBoxContainer.new()
	ch.name = "chips"
	ch.add_theme_constant_override("separation", 4)
	v.add_child(ch)
	for c in md[5]:
		ch.add_child(K.chip(c, K.MUTED))
	return pc

func _show_mode_cards(sel: String) -> void:
	for key in cards:
		var pc: PanelContainer = cards[key]
		var on: bool = key == sel
		pc.add_theme_stylebox_override("panel", K.panel_style(Color(K.ACCENT, 0.10) if on else K.BG1, 12, 16,
			Color(K.ACCENT, 0.9) if on else K.BORDER))
		(pc.find_child("title", true, false) as Label).add_theme_color_override("font_color", K.ACCENT if on else K.TEXT)
		for c in pc.find_child("chips", true, false).get_children():
			K.set_chip_color(c, K.ACCENT if on else K.MUTED)

# ------------------------------------------------------------------ 개발/시험: 조건 편집 (왼쪽)
func _build_cond_left() -> void:
	var v := K.card(left_cond)
	cond_title = K.section(v, K.I_SLIDERS, "실행 조건", K.TEXT)
	var h := HBoxContainer.new()
	h.add_theme_constant_override("separation", 6)
	v.add_child(h)
	preset_ob = OptionButton.new()
	preset_ob.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	preset_ob.focus_mode = Control.FOCUS_NONE
	preset_ob.item_selected.connect(func(i):
		if i > 0:
			action.emit("preset", i - 1)
		preset_ob.select(0))
	h.add_child(preset_ob)
	var ld := K.btn(K.ic(K.I_IMPORT) + "  YAML 불러오기", "회귀 시나리오나 시험 결과 폴더의 scenario.yaml → 같은 조건 재현")
	ld.pressed.connect(func(): yaml_dialog.popup_centered_ratio(0.6))
	h.add_child(ld)
	var df := K.btn(K.ic(K.I_UNDO) + "  기본값", "config/sim.yaml 값으로 되돌리기")
	df.pressed.connect(func(): action.emit("defaults", null))
	h.add_child(df)
	yaml_dialog = FileDialog.new()
	yaml_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	yaml_dialog.access = FileDialog.ACCESS_FILESYSTEM
	yaml_dialog.use_native_dialog = true
	yaml_dialog.title = "시나리오 YAML 선택"
	yaml_dialog.filters = PackedStringArray(["*.yaml, *.yml ; 시나리오 YAML"])
	yaml_dialog.file_selected.connect(func(p): action.emit("load_yaml", p))
	add_child(yaml_dialog)
	var tabs := HBoxContainer.new()
	tabs.add_theme_constant_override("separation", 4)
	left_cond.add_child(tabs)
	var sc := ScrollContainer.new()
	sc.size_flags_vertical = Control.SIZE_EXPAND_FILL
	sc.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	left_cond.add_child(sc)
	var body := VBoxContainer.new()
	body.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	sc.add_child(body)
	form = Form.new(self, tabs, body, sc)
	form.tab_style = "seg"
	form.edited.connect(func(k, x): edited.emit(k, x))
	form.action.connect(func(n, a): action.emit(n, a))

# ------------------------------------------------------------------ 개발/시험: 요약 · 시작 (오른쪽)
func _build_cond_right() -> void:
	var v := K.card(right_cond)
	K.section(v, K.I_INFO, "요약", K.MUTED)
	summary_box = VBoxContainer.new()
	summary_box.add_theme_constant_override("separation", 6)
	v.add_child(summary_box)
	ros_box = K.card(right_cond)
	start_btn = K.primary(K.btn(""))
	start_btn.custom_minimum_size = Vector2(0, 52)
	start_btn.add_theme_font_size_override("font_size", 18)
	start_btn.pressed.connect(func(): action.emit("start", null))
	right_cond.add_child(start_btn)
	last_box = VBoxContainer.new()
	right_cond.add_child(last_box)

# ------------------------------------------------------------------ 회귀: 시나리오 목록 (왼쪽)
func _build_reg_left() -> void:
	var v := K.card(left_reg)
	reg_title = K.section(v, K.I_LIST_CHECK, "시나리오", K.TEXT)
	var h := HBoxContainer.new()
	v.add_child(h)
	for it in [["전체 선택", "regress_all"], ["전체 해제", "regress_none"], [K.ic(K.I_FOLDER) + "  폴더", "open_scenarios"]]:
		var b := K.btn(it[0])
		var a: String = it[1]
		b.pressed.connect(func(): action.emit(a, null))
		h.add_child(b)
	var sc := ScrollContainer.new()
	sc.size_flags_vertical = Control.SIZE_EXPAND_FILL
	sc.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	left_reg.add_child(sc)
	reg_list = VBoxContainer.new()
	reg_list.add_theme_constant_override("separation", 8)
	reg_list.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	sc.add_child(reg_list)

func _fill_reg_list(scs: Array) -> void:
	for c in reg_list.get_children():
		c.queue_free()
	var nsel := 0
	for s in scs:
		nsel += 1 if s["sel"] else 0
		var v := K.card(reg_list)
		var h := HBoxContainer.new()
		v.add_child(h)
		var cb := CheckBox.new()
		cb.text = str(s["name"])
		cb.focus_mode = Control.FOCUS_NONE
		cb.button_pressed = s["sel"]
		cb.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		var path: String = s["path"]
		cb.toggled.connect(func(_on): action.emit("regress_toggle", path))
		h.add_child(cb)
		if s["user"]:
			h.add_child(K.chip("사용자", K.TEAL))
		h.add_child(K.label("%s s" % str(s["duration"]) if s["duration"] != null else "- s", K.MUTED, 13))
		if str(s["desc"]) != "":
			var d := K.label(str(s["desc"]), K.MUTED, 12)
			d.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			v.add_child(d)
	reg_title.text = K.ic(K.I_LIST_CHECK) + "  시나리오 (%d/%d)" % [nsel, scs.size()]

# ------------------------------------------------------------------ 회귀: 실행 (오른쪽)
func _build_reg_right() -> void:
	var v := K.card(right_reg)
	K.section(v, K.I_SLIDERS, "실행 방식", K.TEXT)
	reg_ros = CheckBox.new()
	reg_ros.text = "ROS 연동 (실시간, /cmd_vel · 평가 토픽)"
	reg_ros.focus_mode = Control.FOCUS_NONE
	reg_ros.toggled.connect(func(on): edited.emit("regress.ros", on))
	v.add_child(reg_ros)
	var n := K.label("끄면 단독 고속 실행 (비콘은 시나리오의 beacon_script)", K.MUTED, 12)
	n.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	v.add_child(n)
	reg_run = K.primary(K.btn(""))
	reg_run.custom_minimum_size = Vector2(0, 52)
	reg_run.add_theme_font_size_override("font_size", 18)
	reg_run.pressed.connect(func():
		var rr = m.get("regress")
		action.emit("regress_stop" if typeof(rr) == TYPE_DICTIONARY and rr["running"] else "regress_run", null))
	right_reg.add_child(reg_run)
	reg_prog = VBoxContainer.new()
	right_reg.add_child(reg_prog)

func _fill_reg_prog(rr) -> void:
	for c in reg_prog.get_children():
		c.queue_free()
	if typeof(rr) != TYPE_DICTIONARY:
		return
	var v := K.card(reg_prog)
	if rr["running"]:
		K.section(v, K.I_SPINNER, "실행 중 %d/%d · %d s" % [int(rr["done"]), int(rr["total"]), int(rr["elapsed"])], K.WARN)
	else:
		var ok: bool = rr["code"] == 0
		K.section(v, K.I_FLAG, "완료 · 통과 %d/%d" % [int(rr["npass"]), rr["rows"].size()], K.OK if ok else K.DANGER)
	var pb := ProgressBar.new()
	pb.show_percentage = false
	pb.custom_minimum_size = Vector2(0, 6)
	pb.max_value = maxf(float(rr["total"]), 1.0)
	pb.value = float(rr["done"])
	v.add_child(pb)
	for r in rr["rows"]:
		var h := HBoxContainer.new()
		v.add_child(h)
		var st: String = r[1]
		h.add_child(K.chip(st, {"PASS": K.OK, "FAIL": K.DANGER}.get(st, K.WARN)))
		h.add_child(K.label(str(r[0])))
		if str(r[2]) != "":
			var d := K.label(str(r[2]), K.MUTED, 12)
			d.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			v.add_child(d)
	if not rr["running"]:
		var h := HBoxContainer.new()
		v.add_child(h)
		if rr["report"] != null:
			var b := K.btn(K.ic(K.I_FILE) + "  report.md")
			var p: String = rr["report"]
			b.pressed.connect(func(): action.emit("open", p))
			h.add_child(b)
		var f := K.btn(K.ic(K.I_FOLDER) + "  결과 폴더")
		var o: String = rr["out"]
		f.pressed.connect(func(): action.emit("open", o))
		h.add_child(f)
		if str(rr["err"]) != "":
			var e := K.label("오류: " + str(rr["err"]), K.DANGER, 12)
			e.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			v.add_child(e)

# ------------------------------------------------------------------ 첫 실행: 로봇 모델 (STL)
func _build_robot_modal() -> void:
	robot_layer = Control.new()
	robot_layer.set_anchors_preset(Control.PRESET_FULL_RECT)
	robot_layer.visible = false
	add_child(robot_layer)
	var dim := ColorRect.new()
	dim.color = Color(0, 0, 0, 0.55)
	dim.set_anchors_preset(Control.PRESET_FULL_RECT)
	robot_layer.add_child(dim)
	var cc := CenterContainer.new()
	cc.set_anchors_preset(Control.PRESET_FULL_RECT)
	robot_layer.add_child(cc)
	var pc := PanelContainer.new()
	pc.add_theme_stylebox_override("panel", K.panel_style(K.BG1, 14, 20, K.BORDER))
	pc.custom_minimum_size = Vector2(580, 0)
	cc.add_child(pc)
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 10)
	pc.add_child(v)
	var t := K.label(K.ic(K.I_TOWER) + "  비콘 로봇 모델이 필요합니다", K.WARN, 20)
	t.set_meta("bold", true)
	v.add_child(t)
	var d := K.label("비콘 CAD(STL, 단위 mm)를 선택하면 로봇 메시와 센서 장착 포즈를 만들어 저장합니다. "
		+ "CAD는 공개 배포하지 않으므로 팀 내부에서 받으세요.", K.TEXT, 14)
	d.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	v.add_child(d)
	robot_btn = K.primary(K.btn(""))
	robot_btn.custom_minimum_size = Vector2(0, 44)
	robot_btn.pressed.connect(func(): stl_dialog.popup_centered_ratio(0.6))
	v.add_child(robot_btn)
	robot_log = K.label("", K.MUTED, 12)
	robot_log.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	v.add_child(robot_log)
	stl_dialog = FileDialog.new()
	stl_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	stl_dialog.access = FileDialog.ACCESS_FILESYSTEM
	stl_dialog.use_native_dialog = true
	stl_dialog.title = "비콘 CAD (STL) 선택"
	stl_dialog.filters = PackedStringArray(["*.stl, *.STL ; STL"])
	stl_dialog.current_dir = OS.get_environment("HOME")
	stl_dialog.file_selected.connect(func(p): action.emit("build_robot", p))
	add_child(stl_dialog)

func set_logs(lines: Array) -> void:
	robot_log.text = "\n".join(PackedStringArray(lines.slice(maxi(lines.size() - 6, 0))))

# ------------------------------------------------------------------ 상태 반영
func set_tabs(tabs: Array) -> void:
	form.set_tabs(tabs)

func set_values(v: Dictionary) -> void:
	form.set_values(v)
	reg_ros.set_pressed_no_signal(bool(v.get("regress.ros", false)))

func _changed(key: String, val: Variant) -> bool:
	var s := JSON.stringify(val)
	if sigs.get(key) == s:
		return false
	sigs[key] = s
	return true

func set_mode(mm: Dictionary) -> void:
	m = mm
	var sel: String = m["mode_sel"]
	var reg := sel == "regress"
	_show_mode_cards(sel)
	left_cond.visible = not reg
	right_cond.visible = not reg
	left_reg.visible = reg
	right_reg.visible = reg
	var prev = m.get("prev_mode")
	back_btn.visible = prev != null
	if prev != null:
		back_btn.text = K.ic(K.I_ARROW_LEFT) + "  %s 화면으로 돌아가기" % m["mode_ko"].get(str(prev), "")
	cond_title.text = K.ic(K.I_SLIDERS) + "  " + ("시험 조건" if sel == "test" else "실행 조건")
	if _changed("presets", m["presets"]):
		preset_ob.clear()
		preset_ob.add_item("프리셋 선택...")
		for p in m["presets"]:
			preset_ob.add_item("%s  —  %s" % [p[0], p[1]])
		preset_ob.select(0)
	if _changed("summary", m["summary"]):
		for c in summary_box.get_children():
			c.queue_free()
		for r in m["summary"]:
			var h := HBoxContainer.new()
			h.add_theme_constant_override("separation", 10)
			summary_box.add_child(h)
			var ic := K.label(K.ic(K.ICONS.get(str(r[0]), K.I_INFO)), K.ACCENT, 14)
			ic.custom_minimum_size = Vector2(22, 0)
			h.add_child(ic)
			var k := K.label(str(r[1]), K.MUTED, 14)
			k.custom_minimum_size = Vector2(36, 0)
			h.add_child(k)
			var t := K.label(str(r[2]), K.TEXT, 14)
			t.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			t.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			h.add_child(t)
	if _changed("ros", [m["ros"], sel]):
		for c in ros_box.get_children():
			c.queue_free()
		var h := HBoxContainer.new()
		ros_box.add_child(h)
		h.add_child(K.chip(K.ic(K.I_LINK) + ("  ROS2 연동" if m["ros"] else "  ROS2 없음 (단독 실행)"),
			K.ACCENT if m["ros"] else K.MUTED))
		h.add_child(K.chip(K.ic(K.I_GAMEPAD) + "  비콘 조작: /cmd_vel → 키보드", K.MUTED))
		var notes := ["시험 중 조건은 잠깁니다. 바꾸면 \"이 조건으로 재시작\".", "키보드 조작은 \"수동 개입\"으로 리포트에 기록됩니다."] \
			if sel == "test" else ["개발 모드: 조건을 바꾸고 \"적용\"을 누르면 바로 반영됩니다."]
		for n in notes:
			var l := K.label(n, K.MUTED, 12)
			l.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			ros_box.add_child(l)
	var busy: bool = m["building"]
	start_btn.disabled = busy or not m["robot_ready"]
	start_btn.text = "준비 중..." if busy else (K.ic(K.I_FLAG) + "  시험 시작" if sel == "test" else K.ic(K.I_PLAY) + "  개발 시작")
	if _changed("last", m.get("result")):
		for c in last_box.get_children():
			c.queue_free()
		var r = m.get("result")
		if typeof(r) == TYPE_DICTIONARY:
			var v := K.card(last_box)
			var ok: bool = r["ok"]
			K.section(v, K.I_FLAG, "최근 시험 — %s" % r["name"], K.OK if ok else K.DANGER)
			var h := HBoxContainer.new()
			v.add_child(h)
			h.add_child(K.chip("PASS" if ok else "FAIL", K.OK if ok else K.DANGER))
			if not r["complete"]:
				h.add_child(K.chip("시험 시간 전 종료", K.WARN))
			h.add_child(K.label(str(r["reason"]), K.MUTED, 13))
			var h2 := HBoxContainer.new()
			v.add_child(h2)
			var f := K.btn(K.ic(K.I_FOLDER) + "  결과 폴더")
			var d: String = r["dir"]
			f.pressed.connect(func(): action.emit("open", d))
			h2.add_child(f)
			var more := K.btn(K.ic(K.I_FILE) + "  자세히")
			more.pressed.connect(func(): action.emit("show_result", null))
			h2.add_child(more)
	if reg and _changed("scenarios", m.get("scenarios", [])):
		_fill_reg_list(m.get("scenarios", []))
	var rr = m.get("regress")
	var running: bool = typeof(rr) == TYPE_DICTIONARY and rr["running"]
	var nsel := 0
	for s in m.get("scenarios", []):
		nsel += 1 if s["sel"] else 0
	reg_run.text = K.ic(K.I_STOP) + "  중지" if running else K.ic(K.I_PLAY) + "  %d개 실행" % nsel
	reg_run.disabled = not running and nsel == 0
	reg_ros.disabled = running or not m["ros"]
	if _changed("regress", rr):
		_fill_reg_prog(rr)
	robot_layer.visible = not m["robot_ready"]
	robot_btn.disabled = m["robot_busy"]
	robot_btn.text = "생성 중..." if m["robot_busy"] else K.ic(K.I_FOLDER) + "  STL 파일 선택"
	form.apply_bold(self)
