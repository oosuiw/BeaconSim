extends PanelContainer
## 왼쪽 사이드바: 머리글(모드·홈) + 시험 진행 카드 + 변경 배너 + 탭(맵/시나리오/교통/환경/보기/저장|시험) + 이벤트 피드.
## 탭 내용은 form.gd 가 백엔드 정의로 그린다. 값을 바꾸면 edited(k, v), 버튼은 action(name, arg).

signal edited(k: String, v: Variant)
signal action(name: String, arg: Variant)

const K = preload("res://ui_kit.gd")
const Form = preload("res://form.gd")
const WIDTH := 380.0

var form: Form
var font_bold: Font
var scroll: ScrollContainer
var tab_grid: GridContainer
var status_box: VBoxContainer
var status_sig := ""
var prog: ProgressBar
var prog_lbl: Label
var feed: RichTextLabel
var feed_lines: Array = []
var feed_filter := 0
var feed_seg: Array = []
var version_lbl: Label
var mode_chip: Label
var home_btn: Button

func _init() -> void:
	custom_minimum_size = Vector2(WIDTH, 0)
	K.eat_mouse(self)                                  # 사이드바 위 휠·드래그가 3D 시점으로 새지 않게
	add_theme_stylebox_override("panel", K.panel_style(Color(0.063, 0.075, 0.098, 0.97), 0, 14))
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 8)
	add_child(v)
	# 머리글
	var h := HBoxContainer.new()
	h.add_theme_constant_override("separation", 8)
	v.add_child(h)
	h.add_child(K.label(K.ic(K.I_TOWER), K.ACCENT, 22))
	var title := K.label("BeaconSim", K.TEXT, 22)
	title.set_meta("bold", true)
	h.add_child(title)
	version_lbl = K.label("", K.MUTED, 13)
	version_lbl.size_flags_vertical = Control.SIZE_SHRINK_END
	h.add_child(version_lbl)
	mode_chip = K.chip("", K.ACCENT)
	mode_chip.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	h.add_child(mode_chip)
	var sp := Control.new()
	sp.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	h.add_child(sp)
	home_btn = K.btn(K.ic(K.I_HOUSE), "시작 화면 (모드 변경 · 조건 설정 · 회귀 실행)")
	home_btn.pressed.connect(func(): action.emit("home", null))
	h.add_child(home_btn)
	# 시험 진행 · 변경 배너
	status_box = VBoxContainer.new()
	status_box.add_theme_constant_override("separation", 8)
	v.add_child(status_box)
	# 탭 버튼 (3 x 2)
	tab_grid = GridContainer.new()
	tab_grid.columns = 3
	tab_grid.add_theme_constant_override("h_separation", 6)
	tab_grid.add_theme_constant_override("v_separation", 6)
	v.add_child(tab_grid)
	# 탭 내용
	scroll = ScrollContainer.new()
	scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	v.add_child(scroll)
	var body := VBoxContainer.new()
	body.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	scroll.add_child(body)
	form = Form.new(self, tab_grid, body, scroll)
	form.edited.connect(func(k, x): edited.emit(k, x))
	form.action.connect(func(n, a): action.emit(n, a))
	# 이벤트 피드
	var fc := K.card(v)
	fc.get_parent().custom_minimum_size = Vector2(0, 230)
	var fh := HBoxContainer.new()
	fc.add_child(fh)
	var sl := K.section(fh, K.I_LIST, "이벤트", K.TEXT)
	sl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	for i in range(2):
		var b := K.btn(["전체", "차량"][i], "전체 로그 / 차량 이벤트(추돌, SCC 인식)만" if i == 1 else "", true)
		b.add_theme_font_size_override("font_size", 12)
		b.pressed.connect(func(): _set_feed_filter(i, true))
		feed_seg.append(b)
		fh.add_child(b)
	feed = RichTextLabel.new()
	feed.bbcode_enabled = true
	feed.scroll_following = true
	feed.size_flags_vertical = Control.SIZE_EXPAND_FILL
	feed.selection_enabled = true
	feed.focus_mode = Control.FOCUS_NONE
	feed.add_theme_font_size_override("normal_font_size", 12)
	feed.add_theme_constant_override("line_separation", 3)
	fc.add_child(feed)
	feed_seg[0].set_pressed_no_signal(true)

# 예전 이름 호환 (client.gd · selftest.gd)
var vals: Dictionary:
	get: return form.vals
var binds: Dictionary:
	get: return form.binds
var tab_btns: Array:
	get: return form.tab_btns

func set_fonts(bold: Font) -> void:
	font_bold = bold
	form.font_bold = bold
	form.apply_bold(self)

func set_version(v: String) -> void:
	version_lbl.text = "v" + v

func set_tabs(tabs: Array) -> void:
	form.set_tabs(tabs)

func select_tab(i: int) -> void:
	form.select_tab(i)

func set_values(v: Dictionary, force := false) -> void:
	form.set_values(v, force)

func set_widget(k: String, x: Variant) -> void:
	form.set_widget(k, x)

func set_info(d: Dictionary) -> void:
	form.set_info(d)

# ------------------------------------------------------------------ 모드 · 시험 카드 · 변경 배너
func set_mode(m: Dictionary) -> void:
	var mode = m.get("mode")
	var test: bool = mode == "test"
	mode_chip.text = (K.ic(K.I_CLIPBOARD) if test else K.ic(K.I_WRENCH)) + "  " + str(m["mode_ko"].get(str(mode), ""))
	K.set_chip_color(mode_chip, K.WARN if test else K.ACCENT)
	var sig := JSON.stringify([mode, m.get("session"), m.get("changes"), m.get("building")])
	if sig == status_sig:
		return
	status_sig = sig
	for c in status_box.get_children():
		c.queue_free()
	prog = null
	prog_lbl = null
	var s = m.get("session")
	if test and typeof(s) == TYPE_DICTIONARY:
		var v := K.card(status_box)
		if s["done"]:
			var ok: bool = s["ok"] == true
			K.section(v, K.I_FLAG, "%s — 종료" % s["name"], K.OK if ok else K.DANGER)
			var h := HBoxContainer.new()
			v.add_child(h)
			var chip := K.chip("PASS" if ok else "FAIL", K.OK if ok else K.DANGER)
			chip.set_meta("bold", true)
			h.add_child(chip)
			var sp := Control.new()
			sp.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			h.add_child(sp)
			var r := K.btn(K.ic(K.I_FILE) + "  결과")
			r.pressed.connect(func(): action.emit("show_result", null))
			h.add_child(r)
			var again := K.primary(K.btn(K.ic(K.I_ROTATE) + "  다시 시험"))
			again.pressed.connect(func(): action.emit("restart", null))
			h.add_child(again)
		else:
			K.section(v, K.I_CLIPBOARD, "시험 중 · %s" % s["name"], K.WARN)
			if float(s["duration"]) > 0:
				prog = ProgressBar.new()
				prog.show_percentage = false
				prog.custom_minimum_size = Vector2(0, 6)
				prog.max_value = float(s["duration"])
				var bg := StyleBoxFlat.new()
				bg.bg_color = K.BG3
				bg.set_corner_radius_all(3)
				var fg := bg.duplicate() as StyleBoxFlat
				fg.bg_color = K.WARN
				prog.add_theme_stylebox_override("background", bg)
				prog.add_theme_stylebox_override("fill", fg)
				v.add_child(prog)
			var h := HBoxContainer.new()
			v.add_child(h)
			prog_lbl = K.label("", K.MUTED, 13)
			prog_lbl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			prog_lbl.set_meta("dur", float(s["duration"]))
			h.add_child(prog_lbl)
			var n_iv := int(s["n_iv"])
			var iv := K.chip(K.ic(K.I_HAND) + "  수동 개입 %d" % n_iv, K.WARN if n_iv else K.MUTED)
			iv.tooltip_text = "방향키로 비콘을 움직인 횟수 (리포트에 기록, 회귀 실행에서는 재현되지 않음)"
			iv.mouse_filter = Control.MOUSE_FILTER_PASS
			h.add_child(iv)
			var stop := K.btn(K.ic(K.I_STOP) + "  시험 종료")
			stop.pressed.connect(func(): action.emit("finish_test", null))
			v.add_child(stop)
	var ch: Array = m.get("changes", [])
	if ch.size() > 0 and mode in ["dev", "test"]:
		var v := K.card(status_box)
		if test:
			K.section(v, K.I_LOCK, "시험 조건 잠김 · 변경 %d개" % ch.size(), K.WARN)
			v.add_child(K.label("지금 시험에는 반영되지 않습니다", K.MUTED, 12))
		else:
			K.section(v, K.I_PEN, "변경 사항 %d개" % ch.size(), K.ACCENT)
		var shown := ch.slice(0, 5)
		var txt := ", ".join(PackedStringArray(shown)) + (" 외 %d개" % (ch.size() - 5) if ch.size() > 5 else "")
		var l := K.label(txt, K.MUTED, 12)
		l.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		v.add_child(l)
		var h := HBoxContainer.new()
		v.add_child(h)
		var ap := K.btn(K.ic(K.I_ROTATE) + "  " + ("이 조건으로 재시작" if test else "적용 (재시작)"))
		K.primary(ap)
		if test:
			ap.add_theme_stylebox_override("normal", K.panel_style(Color(K.WARN, 0.9), 8, 10))
			ap.add_theme_stylebox_override("hover", K.panel_style(K.WARN, 8, 10))
		ap.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		ap.size_flags_stretch_ratio = 1.8
		ap.disabled = m.get("building", false)
		ap.pressed.connect(func(): action.emit("restart", null))
		h.add_child(ap)
		var rv := K.btn(K.ic(K.I_UNDO) + "  되돌리기")
		rv.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		rv.pressed.connect(func(): action.emit("revert", null))
		h.add_child(rv)
	form.apply_bold(status_box)

## 매 상태마다: 시험 진행 막대 · 시간
func set_time(t: float) -> void:
	if prog_lbl == null:
		return
	var dur: float = prog_lbl.get_meta("dur", 0.0)
	if prog:
		prog.value = minf(t, dur)
	prog_lbl.text = ("%.0f / %.0f s" % [t, dur]) if dur > 0 else ("%.0f s (제한 없음)" % t)

# ------------------------------------------------------------------ 이벤트 피드
func add_log(line: String) -> void:
	feed_lines.append(line)
	if feed_lines.size() > 300:
		feed_lines = feed_lines.slice(feed_lines.size() - 300)
		_set_feed_filter(feed_filter, false)
		return
	if _passes(line):
		feed.append_text(_fmt_line(line))

func _passes(line: String) -> bool:
	return feed_filter == 0 or line.contains("차량 #")

func _set_feed_filter(i: int, user: bool) -> void:
	feed_filter = i
	for k in range(feed_seg.size()):
		feed_seg[k].set_pressed_no_signal(k == i)
	feed.clear()
	for ln in feed_lines:
		if _passes(ln):
			feed.append_text(_fmt_line(ln))
	if user:
		edited.emit("view.log_filter", i)

static func _fmt_line(ln: String) -> String:
	var sp := ln.find(" ")
	var ts := ln.substr(0, sp) if sp > 0 else ""
	var msg := ln.substr(sp + 1) if sp > 0 else ln
	var icon := K.I_INFO
	var col := K.MUTED
	if msg.contains("추돌"):
		icon = K.I_BURST
		col = K.DANGER
	elif msg.contains("SCC"):
		icon = K.I_GAUGE
		col = K.SCC
	elif msg.contains("PASS"):
		icon = K.I_CHECK
		col = K.OK
	elif msg.contains("실패") or msg.contains("없음") or msg.contains("FAIL") or msg.contains("작음"):
		icon = K.I_WARN
		col = K.WARN
	var tcol := col if col != K.MUTED else K.TEXT
	return "[color=%s]%s[/color] [color=%s]%s[/color]  [color=%s]%s[/color]\n" % [K.hexc(col), K.ic(icon),
		K.hexc(K.MUTED), ts, K.hexc(tcol), msg.replace("[", "[lb]")]
