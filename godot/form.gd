extends RefCounted
## 백엔드 정의(beacon_sim/ui_schema.py)로 탭·카드·입력 항목을 만드는 폼. 사이드바와 시작 화면이 함께 쓴다.
## 값을 바꾸면 edited(k, v), 버튼은 action(name, arg). 값 반영은 set_values / set_widget (신호 없이).
## 항목 종류: int / float / num / bool / choice / seg / text / dir / note / info / button / group / row / presets

signal edited(k: String, v: Variant)
signal action(name: String, arg: Variant)

const K = preload("res://ui_kit.gd")

var host: Control               # 파일 대화상자를 붙일 곳
var tab_box: Container          # 탭 버튼 자리 (GridContainer / HBoxContainer)
var body: VBoxContainer         # 탭 내용 자리
var scroll: ScrollContainer     # 있으면 탭 바꿀 때 맨 위로
var tab_style := "grid"         # grid (사이드바 아이콘 탭) | seg (시작 화면 가로 탭)
var tab_btns: Array = []
var tab_bodies: Array = []
var cur_tab := 0
var binds := {}                 # 키 → [{t, w, ...}]
var enables: Array = []         # [{cond, ws}]
var busy_btns: Array = []
var info_boxes := {}
var info := {}
var vals := {}
var editing_key := ""
var edit_t := 0.0
var dir_dialog: FileDialog
var dir_key := ""
var font_bold: Font

func _init(host_: Control, tab_box_: Container, body_: VBoxContainer, scroll_: ScrollContainer = null) -> void:
	host = host_
	tab_box = tab_box_
	body = body_
	scroll = scroll_
	dir_dialog = FileDialog.new()
	dir_dialog.file_mode = FileDialog.FILE_MODE_OPEN_DIR
	dir_dialog.access = FileDialog.ACCESS_FILESYSTEM
	dir_dialog.use_native_dialog = true
	dir_dialog.title = "저장 위치 선택"
	dir_dialog.dir_selected.connect(func(p): _emit(dir_key, p))
	host.add_child(dir_dialog)

# ------------------------------------------------------------------ 탭
func set_tabs(tabs: Array) -> void:
	var sc := scroll.scroll_vertical if scroll else 0
	var cur_id := ""
	if cur_tab < tab_bodies.size():
		cur_id = str(tab_bodies[cur_tab].get_meta("id", ""))
	for c in tab_box.get_children():
		c.queue_free()
	for c in body.get_children():
		c.queue_free()
	tab_btns.clear()
	tab_bodies.clear()
	binds.clear()
	enables.clear()
	busy_btns.clear()
	info_boxes.clear()
	var keep := 0
	for i in range(tabs.size()):
		var t: Dictionary = tabs[i]
		if str(t["id"]) == cur_id:
			keep = i
		var b: Button
		if tab_style == "grid":
			b = K.btn(K.ic(int(t["icon"])) + "  " + str(t["label"]), "", true)
			b.custom_minimum_size = Vector2(0, 34)
			b.add_theme_stylebox_override("pressed", K.panel_style(Color(K.ACCENT, 0.18), 8, 10))
			b.add_theme_stylebox_override("hover_pressed", K.panel_style(Color(K.ACCENT, 0.28), 8, 10))
			b.add_theme_stylebox_override("normal", K.panel_style(K.BG1, 8, 10))
			b.add_theme_color_override("font_color", K.MUTED)
			b.add_theme_color_override("font_pressed_color", K.ACCENT)
			b.add_theme_color_override("font_hover_pressed_color", K.ACCENT)
		else:
			b = K.btn(str(t["label"]), "", true)
		b.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		b.pressed.connect(func(): select_tab(i))
		tab_box.add_child(b)
		tab_btns.append(b)
		var tb := VBoxContainer.new()
		tb.add_theme_constant_override("separation", 8)
		tb.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		tb.set_meta("id", str(t["id"]))
		body.add_child(tb)
		tab_bodies.append(tb)
		for c in t["cards"]:
			card_or_item(tb, c)
	select_tab(keep, false)
	set_values(vals, true)
	set_info(info)
	apply_bold(host)
	if scroll:
		await host.get_tree().process_frame
		scroll.scroll_vertical = sc

func select_tab(i: int, reset_scroll := true) -> void:
	cur_tab = i
	for k in range(tab_btns.size()):
		tab_btns[k].set_pressed_no_signal(k == i)
		tab_bodies[k].visible = k == i
	if reset_scroll and scroll:
		scroll.scroll_vertical = 0

func apply_bold(n: Node) -> void:
	if font_bold == null:
		return
	for c in n.get_children():
		if c is Label and c.get_meta("bold", false):
			c.add_theme_font_override("font", font_bold)
		apply_bold(c)

# ------------------------------------------------------------------ 항목
func card_or_item(parent: Control, c: Dictionary) -> void:
	if c.has("items") and c.has("title"):
		var v := K.card(parent)
		var col: Color = K.NAMED.get(c.get("color", "text"), K.TEXT)
		K.section(v, int(c["icon"]), str(c["title"]), col)
		for it in c["items"]:
			item(v, it)
	else:
		item(parent, c)

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

static func fmt(f: String, v: float) -> String:
	if f.contains("%d"):
		return f % int(round(v))
	return f % v

func _width(w: Control, it: Dictionary) -> void:
	if it.has("w"):
		w.custom_minimum_size.x = float(it["w"])
	elif it.get("expand", false):
		w.size_flags_horizontal = Control.SIZE_EXPAND_FILL

func _spin(it: Dictionary, k: String) -> SpinBox:
	var sp := SpinBox.new()
	sp.min_value = float(it.get("min", 0))
	sp.max_value = float(it.get("max", 1e9))
	sp.step = float(it.get("step", 1))
	sp.suffix = str(it.get("suffix", ""))
	sp.select_all_on_focus = true
	sp.custom_arrow_step = sp.step
	sp.get_line_edit().text_submitted.connect(func(_s): sp.get_line_edit().release_focus())
	sp.value_changed.connect(func(x): _emit(k, int(x) if sp.step >= 1 and it.get("int", sp.step >= 1) else x))
	return sp

func item(parent: Control, it: Dictionary) -> void:
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
				var sp := _spin(it, k)
				sp.rounded = true
				sp.custom_minimum_size = Vector2(150, 0)
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
			var f: String = it.get("fmt", "%d")
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
				vl.text = fmt(f, x)
				_emit(k, int(round(x)) if is_int else x))
			v.add_child(s)
			_bind(k, {"t": "slider", "w": s, "vl": vl, "fmt": f})
			_enable(it, [s, l])
			_tip(it, [l, s])
		"num":
			var sp := _spin(it, k)
			_width(sp, it)
			parent.add_child(sp)
			_bind(k, {"t": "spin", "w": sp})
			_enable(it, [sp])
			_tip(it, [sp])
		"bool":
			var cb := CheckBox.new()
			cb.text = str(it["label"])
			cb.focus_mode = Control.FOCUS_NONE
			cb.toggled.connect(func(on): _emit(k, on))
			_width(cb, it)
			parent.add_child(cb)
			_bind(k, {"t": "check", "w": cb})
			_enable(it, [cb])
			_tip(it, [cb])
		"choice":
			var ob := OptionButton.new()
			ob.focus_mode = Control.FOCUS_NONE
			var opts: Array = it["opts"]
			for o in opts:
				ob.add_item(str(o[1]))
			ob.item_selected.connect(func(i): _emit(k, opts[i][0]))
			if it.has("label"):
				var h := HBoxContainer.new()
				parent.add_child(h)
				var l := K.label(str(it["label"]))
				l.custom_minimum_size = Vector2(110, 0)
				h.add_child(l)
				ob.size_flags_horizontal = Control.SIZE_EXPAND_FILL
				h.add_child(ob)
				_tip(it, [l, ob])
			else:
				_width(ob, it)
				parent.add_child(ob)
			_bind(k, {"t": "choice", "w": ob, "opts": opts})
			_enable(it, [ob])
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
		"text":
			var h := HBoxContainer.new()
			parent.add_child(h)
			if it.has("label"):
				var l := K.label(str(it["label"]))
				l.custom_minimum_size = Vector2(80, 0)
				h.add_child(l)
			var le := LineEdit.new()
			le.size_flags_horizontal = Control.SIZE_EXPAND_FILL
			le.placeholder_text = str(it.get("hint", ""))
			le.text_submitted.connect(func(s):
				le.release_focus()
				if vals.get(k) != s: _emit(k, s))
			le.focus_exited.connect(func(): if vals.get(k) != le.text: _emit(k, le.text))
			h.add_child(le)
			_bind(k, {"t": "line", "w": le})
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
			if parent is HBoxContainer:
				l.autowrap_mode = TextServer.AUTOWRAP_OFF
				l.size_flags_vertical = Control.SIZE_SHRINK_CENTER
			parent.add_child(l)
		"info":
			var v := VBoxContainer.new()
			v.add_theme_constant_override("separation", 2)
			parent.add_child(v)
			info_boxes[str(it["id"])] = v
		"button":
			var txt := K.ic(int(it.get("icon", K.I_CIRCLE)))
			if str(it.get("label", "")) != "":
				txt += "  " + str(it["label"])
			var b := K.btn(txt)
			b.custom_minimum_size = Vector2(0, 32 if parent is HBoxContainer else 36)
			_width(b, it)
			if it.get("primary", false):
				K.primary(b)
			var name: String = it["action"]
			var arg = it.get("arg")
			b.pressed.connect(func(): action.emit(name, arg))
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
				item(v, c)
		"row":
			var h := HBoxContainer.new()
			h.add_theme_constant_override("separation", 6)
			parent.add_child(h)
			for c in it["items"]:
				item(h, c)
		"presets":
			var h := HBoxContainer.new()
			parent.add_child(h)
			for p in [["beacon", "비콘  1"], ["overview", "전체  2"], ["tunnel", "터널  3"]]:
				var b := K.btn(p[1])
				b.size_flags_horizontal = Control.SIZE_EXPAND_FILL
				var cam: String = p[0]
				b.pressed.connect(func(): action.emit("cam", cam))
				h.add_child(b)

func _seg_show(bs: Array, opts: Array, v: Variant) -> void:
	for i in range(bs.size()):
		bs[i].set_pressed_no_signal(same(opts[i][0], v))

static func same(a: Variant, b: Variant) -> bool:
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
				if not d["w"].get_line_edit().has_focus():
					d["w"].get_line_edit().text = fmt("%d" if d["w"].step >= 1 else "%.2f", float(x)) + d["w"].suffix
			"slider":
				d["w"].set_value_no_signal(float(x))
				d["vl"].text = fmt(d["fmt"], float(x))
			"check":
				d["w"].set_pressed_no_signal(bool(x))
			"choice":
				for i in range(d["opts"].size()):
					if same(d["opts"][i][0], x):
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
