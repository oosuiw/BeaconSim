extends PanelContainer
## 왼쪽 사이드바: 머리글 + 탭(맵/시나리오/교통/환경/보기/저장) + 이벤트 피드.
## 탭 내용은 백엔드가 보낸 정의(beacon_sim/ui_schema.py)로 그린다. 값을 바꾸면 edited(k, v), 버튼은 action(name).

signal edited(k: String, v: Variant)
signal action(name: String)

const K = preload("res://ui_kit.gd")
const WIDTH := 380.0

var font_bold: Font
var tab_defs: Array = []
var tab_btns: Array = []
var tab_bodies: Array = []
var cur_tab := 0
var scroll: ScrollContainer
var body: VBoxContainer
var tab_grid: GridContainer
var binds := {}                 # 키 → [{t, w, ...}]
var enables: Array = []         # [{cond, ws}]
var busy_btns: Array = []
var info_boxes := {}
var info := {}
var vals := {}
var editing_key := ""
var edit_t := 0.0
var feed: RichTextLabel
var feed_lines: Array = []
var feed_filter := 0
var feed_seg: Array = []
var dir_dialog: FileDialog
var dir_key := ""
var version_lbl: Label

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
	var logo := K.label(K.ic(K.I_TOWER), K.ACCENT, 22)
	h.add_child(logo)
	var title := K.label("BeaconSim", K.TEXT, 22)
	title.set_meta("bold", true)
	h.add_child(title)
	version_lbl = K.label("", K.MUTED, 13)
	version_lbl.size_flags_vertical = Control.SIZE_SHRINK_END
	h.add_child(version_lbl)
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
	body = VBoxContainer.new()
	body.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	scroll.add_child(body)
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
	dir_dialog = FileDialog.new()
	dir_dialog.file_mode = FileDialog.FILE_MODE_OPEN_DIR
	dir_dialog.access = FileDialog.ACCESS_FILESYSTEM
	dir_dialog.use_native_dialog = true
	dir_dialog.title = "저장 위치 선택"
	dir_dialog.dir_selected.connect(func(p): edited.emit(dir_key, p))
	add_child(dir_dialog)

func set_fonts(bold: Font) -> void:
	font_bold = bold
	_apply_bold(self)

func _apply_bold(n: Node) -> void:
	for c in n.get_children():
		if c is Label and c.get_meta("bold", false):
			c.add_theme_font_override("font", font_bold)
		_apply_bold(c)

func set_version(v: String) -> void:
	version_lbl.text = "v" + v

# ------------------------------------------------------------------ 탭
func set_tabs(tabs: Array) -> void:
	var sc := scroll.scroll_vertical
	tab_defs = tabs
	for c in tab_grid.get_children():
		c.queue_free()
	for c in body.get_children():
		c.queue_free()
	tab_btns.clear()
	tab_bodies.clear()
	binds.clear()
	enables.clear()
	busy_btns.clear()
	info_boxes.clear()
	for i in range(tabs.size()):
		var t: Dictionary = tabs[i]
		var b := K.btn(K.ic(int(t["icon"])) + "  " + str(t["label"]), "", true)
		b.custom_minimum_size = Vector2(0, 34)
		b.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		b.add_theme_stylebox_override("pressed", K.panel_style(Color(K.ACCENT, 0.18), 8, 10))
		b.add_theme_stylebox_override("hover_pressed", K.panel_style(Color(K.ACCENT, 0.28), 8, 10))
		b.add_theme_stylebox_override("normal", K.panel_style(K.BG1, 8, 10))
		b.add_theme_color_override("font_color", K.MUTED)
		b.add_theme_color_override("font_pressed_color", K.ACCENT)
		b.add_theme_color_override("font_hover_pressed_color", K.ACCENT)
		b.pressed.connect(func(): select_tab(i))
		tab_grid.add_child(b)
		tab_btns.append(b)
		var tb := VBoxContainer.new()
		tb.add_theme_constant_override("separation", 8)
		tb.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		body.add_child(tb)
		tab_bodies.append(tb)
		for c in t["cards"]:
			_card_or_item(tb, c)
	select_tab(mini(cur_tab, tabs.size() - 1), false)
	set_values(vals, true)
	set_info(info)
	if font_bold:
		_apply_bold(self)
	await get_tree().process_frame
	scroll.scroll_vertical = sc

func select_tab(i: int, reset_scroll := true) -> void:
	cur_tab = i
	for k in range(tab_btns.size()):
		tab_btns[k].set_pressed_no_signal(k == i)
		tab_bodies[k].visible = k == i
	if reset_scroll:
		scroll.scroll_vertical = 0

func _card_or_item(parent: Control, c: Dictionary) -> void:
	if c.has("items"):
		var v := K.card(parent)
		var col: Color = K.NAMED.get(c.get("color", "text"), K.TEXT)
		K.section(v, int(c["icon"]), str(c["title"]), col)
		for it in c["items"]:
			_item(v, it)
	else:
		_item(parent, c)

func _bind(k: String, d: Dictionary) -> void:
	if not binds.has(k):
		binds[k] = []
	binds[k].append(d)

func _enable(it: Dictionary, ws: Array) -> void:
	if it.has("enable"):
		enables.append({"cond": str(it["enable"]), "ws": ws})

func _tip(it: Dictionary, ws: Array) -> void:
	if it.has("tip"):
		for w in ws:
			w.tooltip_text = str(it["tip"])
			if w is Label:
				w.mouse_filter = Control.MOUSE_FILTER_PASS

func _emit(k: String, v: Variant) -> void:
	editing_key = k
	edit_t = Time.get_ticks_msec() / 1000.0
	vals[k] = v
	_update_enables()
	edited.emit(k, v)

func _fmt(fmt: String, v: float) -> String:
	if fmt.contains("%d"):
		return fmt % int(round(v))
	return fmt % v

func _item(parent: Control, it: Dictionary) -> void:
	var t: String = it["t"]
	var k: String = it.get("k", "")
	match t:
		"float", "int":
			if t == "int" and not it.get("slider", false):
				var h := HBoxContainer.new()
				parent.add_child(h)
				var l := K.label(str(it["label"]))
				l.size_flags_horizontal = Control.SIZE_EXPAND_FILL
				h.add_child(l)
				var sp := SpinBox.new()
				sp.min_value = float(it["min"])
				sp.max_value = float(it["max"])
				sp.step = 1
				sp.rounded = true
				sp.custom_minimum_size = Vector2(150, 0)
				sp.select_all_on_focus = true
				sp.get_line_edit().text_submitted.connect(func(_s): sp.get_line_edit().release_focus())
				sp.value_changed.connect(func(x): _emit(k, int(x)))
				h.add_child(sp)
				_bind(k, {"t": "spin", "w": sp})
				_enable(it, [sp])
				_tip(it, [l, sp])
				return
			var v := VBoxContainer.new()
			v.add_theme_constant_override("separation", 1)
			parent.add_child(v)
			var h := HBoxContainer.new()
			v.add_child(h)
			var l := K.label(str(it["label"]))
			l.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			h.add_child(l)
			var fmt: String = it.get("fmt", "%d")
			var vl := K.label("", K.MUTED, 13)
			h.add_child(vl)
			var s := HSlider.new()
			s.min_value = float(it["min"])
			s.max_value = float(it["max"])
			s.step = float(it.get("step", 1.0))
			s.focus_mode = Control.FOCUS_NONE
			s.scrollable = false
			s.custom_minimum_size = Vector2(0, 20)
			var is_int := t == "int"
			s.value_changed.connect(func(x):
				vl.text = _fmt(fmt, x)
				_emit(k, int(round(x)) if is_int else x))
			v.add_child(s)
			_bind(k, {"t": "slider", "w": s, "vl": vl, "fmt": fmt})
			_enable(it, [s, l])
			_tip(it, [l, s])
		"bool":
			var cb := CheckBox.new()
			cb.text = str(it["label"])
			cb.focus_mode = Control.FOCUS_NONE
			cb.toggled.connect(func(on): _emit(k, on))
			parent.add_child(cb)
			_bind(k, {"t": "check", "w": cb})
			_enable(it, [cb])
			_tip(it, [cb])
		"choice":
			var h := HBoxContainer.new()
			parent.add_child(h)
			var l := K.label(str(it["label"]))
			l.custom_minimum_size = Vector2(110, 0)
			h.add_child(l)
			var ob := OptionButton.new()
			ob.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			ob.focus_mode = Control.FOCUS_NONE
			var opts: Array = it["opts"]
			for o in opts:
				ob.add_item(str(o[1]))
			ob.item_selected.connect(func(i): _emit(k, opts[i][0]))
			h.add_child(ob)
			_bind(k, {"t": "choice", "w": ob, "opts": opts})
			_enable(it, [ob])
			_tip(it, [l, ob])
		"seg":
			if it.has("label"):
				parent.add_child(K.label(str(it["label"]), K.MUTED, 13))
			var h := HBoxContainer.new()
			h.add_theme_constant_override("separation", 4)
			parent.add_child(h)
			var opts: Array = it["opts"]
			var bs: Array = []
			for o in opts:
				var b := K.btn(str(o[1]), "", true)
				b.size_flags_horizontal = Control.SIZE_EXPAND_FILL
				var val = o[0]
				b.pressed.connect(func():
					_emit(k, val)
					_seg_show(bs, opts, val))
				h.add_child(b)
				bs.append(b)
			_bind(k, {"t": "seg", "bs": bs, "opts": opts})
			_enable(it, bs)
		"dir":
			parent.add_child(K.label(str(it.get("label", "")), K.MUTED, 13))
			var h := HBoxContainer.new()
			parent.add_child(h)
			var le := LineEdit.new()
			le.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			le.text_submitted.connect(func(s):
				le.release_focus()
				_emit(k, s))
			le.focus_exited.connect(func(): if vals.get(k) != le.text: _emit(k, le.text))
			h.add_child(le)
			var b := K.btn(K.ic(K.I_FOLDER), "폴더 선택")
			b.pressed.connect(func():
				dir_key = k
				dir_dialog.current_dir = le.text
				dir_dialog.popup_centered_ratio(0.6))
			h.add_child(b)
			_bind(k, {"t": "line", "w": le})
		"note":
			var l := K.label(str(it["text"]), K.MUTED, 12)
			l.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			parent.add_child(l)
		"info":
			var v := VBoxContainer.new()
			v.add_theme_constant_override("separation", 2)
			parent.add_child(v)
			info_boxes[str(it["id"])] = v
		"button":
			var b := K.btn(K.ic(int(it.get("icon", K.I_CIRCLE))) + "  " + str(it["label"]))
			b.custom_minimum_size = Vector2(0, 36)
			if it.get("primary", false):
				K.primary(b)
			var name: String = it["action"]
			b.pressed.connect(func(): action.emit(name))
			parent.add_child(b)
			if it.has("busy"):
				busy_btns.append({"w": b, "k": str(it["busy"]), "label": b.text,
					"busy_label": K.ic(int(it.get("icon", K.I_CIRCLE))) + "  " + str(it.get("busy_label", "..."))})
		"group":
			var b := K.btn(K.ic(K.I_CHEV_R) + "  " + str(it["label"]), "", true)
			b.alignment = HORIZONTAL_ALIGNMENT_LEFT
			var flat := StyleBoxEmpty.new()
			for st in ["normal", "hover", "pressed", "hover_pressed"]:
				b.add_theme_stylebox_override(st, flat)
			b.add_theme_color_override("font_color", K.MUTED)
			b.add_theme_color_override("font_pressed_color", K.TEXT)
			b.add_theme_color_override("font_hover_pressed_color", K.TEXT)
			parent.add_child(b)
			var v := VBoxContainer.new()
			v.add_theme_constant_override("separation", 6)
			v.visible = false
			parent.add_child(v)
			var lbl: String = it["label"]
			b.toggled.connect(func(on):
				v.visible = on
				b.text = K.ic(K.I_CHEV_D if on else K.I_CHEV_R) + "  " + lbl)
			for c in it["items"]:
				_item(v, c)
		"presets":
			var h := HBoxContainer.new()
			parent.add_child(h)
			for p in [["beacon", "비콘  1"], ["overview", "전체  2"], ["tunnel", "터널  3"]]:
				var b := K.btn(p[1])
				b.size_flags_horizontal = Control.SIZE_EXPAND_FILL
				var name: String = "preset:" + p[0]
				b.pressed.connect(func(): action.emit(name))
				h.add_child(b)

func _seg_show(bs: Array, opts: Array, v: Variant) -> void:
	for i in range(bs.size()):
		bs[i].set_pressed_no_signal(_same(opts[i][0], v))

static func _same(a: Variant, b: Variant) -> bool:
	if typeof(a) in [TYPE_INT, TYPE_FLOAT] and typeof(b) in [TYPE_INT, TYPE_FLOAT]:
		return absf(float(a) - float(b)) < 1e-6
	return str(a) == str(b)

# ------------------------------------------------------------------ 값
func set_values(v: Dictionary, force := false) -> void:
	for k in v:
		vals[k] = v[k]
	var now := Time.get_ticks_msec() / 1000.0
	for k in binds:
		if not vals.has(k):
			continue
		if not force and k == editing_key and now - edit_t < 0.5:     # 끌고 있는 슬라이더는 건드리지 않음
			continue
		set_widget(k, vals[k])
	_update_enables()
	for bb in busy_btns:
		var busy: bool = vals.get(bb["k"], false)
		bb["w"].disabled = busy
		bb["w"].text = bb["busy_label"] if busy else bb["label"]

## 위젯 하나를 값에 맞춤 (신호 없이)
func set_widget(k: String, x: Variant) -> void:
	vals[k] = x
	for d in binds.get(k, []):
		match d["t"]:
			"spin":
				d["w"].set_value_no_signal(float(x))
				d["w"].get_line_edit().text = str(int(x))
			"slider":
				d["w"].set_value_no_signal(float(x))
				d["vl"].text = _fmt(d["fmt"], float(x))
			"check":
				d["w"].set_pressed_no_signal(bool(x))
			"choice":
				for i in range(d["opts"].size()):
					if _same(d["opts"][i][0], x):
						d["w"].select(i)
			"seg":
				_seg_show(d["bs"], d["opts"], x)
			"line":
				if not d["w"].has_focus():
					d["w"].text = str(x)

func _update_enables() -> void:
	for e in enables:
		var c: String = e["cond"]
		var neg := c.begins_with("!")
		var on := bool(vals.get(c.trim_prefix("!"), true))
		if neg:
			on = not on
		for w in e["ws"]:
			if w is Range:
				w.editable = on
			elif w is BaseButton:
				w.disabled = not on
			if w is Control:
				w.modulate.a = 1.0 if on else 0.45

func set_info(d: Dictionary) -> void:
	info = d
	for id in info_boxes:
		var box: VBoxContainer = info_boxes[id]
		for c in box.get_children():
			c.queue_free()
		var lines: Array = d.get(id, ["-"])
		for i in range(lines.size()):
			var l := K.label(str(lines[i]), K.TEXT if i == 0 else K.MUTED, 14 if i == 0 else 12)
			l.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
			box.add_child(l)

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

func _fmt_line(ln: String) -> String:
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
	elif msg.contains("실패") or msg.contains("없음"):
		icon = K.I_WARN
		col = K.WARN
	var tcol := col if col != K.MUTED else K.TEXT
	return "[color=%s]%s[/color] [color=%s]%s[/color]  [color=%s]%s[/color]\n" % [K.hexc(col), K.ic(icon),
		K.hexc(K.MUTED), ts, K.hexc(tcol), msg.replace("[", "[lb]")]
