extends RefCounted
## UI 공용: 색 팔레트(ui_style.py 와 같음), 테마, 작은 위젯 도우미

const BG0 := Color(0.055, 0.067, 0.086, 0.92)
const BG1 := Color(0.082, 0.098, 0.125, 1.0)
const BG2 := Color(0.118, 0.137, 0.170, 1.0)
const BG3 := Color(0.160, 0.185, 0.225, 1.0)
const BORDER := Color(0.200, 0.227, 0.275, 1.0)
const TEXT := Color(0.902, 0.918, 0.941)
const MUTED := Color(0.545, 0.588, 0.647)
const ACCENT := Color(0.302, 0.639, 1.0)
const TEAL := Color(0.169, 0.831, 0.773)
const OK := Color(0.239, 0.863, 0.518)
const WARN := Color(1.0, 0.722, 0.302)
const DANGER := Color(1.0, 0.353, 0.373)
const SCC := Color(0.780, 0.490, 1.0)
const AMBER := Color(1.0, 0.600, 0.200)
const NAMED := {"text": TEXT, "muted": MUTED, "accent": ACCENT, "teal": TEAL, "ok": OK, "warn": WARN,
	"danger": DANGER, "scc": SCC, "amber": AMBER}

# Font Awesome 6 Solid
const I_PLAY := 0xf04b
const I_PAUSE := 0xf04c
const I_ROTATE := 0xf2f9
const I_MAP := 0xf279
const I_CLOCK := 0xf017
const I_LINK := 0xf0c1
const I_DOT := 0xf192
const I_BULLSEYE := 0xf140
const I_WIFI := 0xf1eb
const I_SAT := 0xf7bf
const I_CAMERA := 0xf030
const I_GAMEPAD := 0xf11b
const I_TOWER := 0xf519
const I_CAR := 0xf5e4
const I_BURST := 0xf5e1
const I_INFO := 0xf05a
const I_XMARK := 0xf00d
const I_LIST := 0xf03a
const I_QUESTION := 0xf059
const I_CIRCLE := 0xf111
const I_CHECK := 0xf058
const I_WARN := 0xf071
const I_GAUGE := 0xf625
const I_HAND := 0xf256
const I_FOLDER := 0xf07c
const I_CHEV_R := 0xf054
const I_CHEV_D := 0xf078
const I_MOON := 0xf186
const I_SUN := 0xf185
const I_CLOUD_SUN := 0xf6c4
const I_RAIN := 0xf73d
const I_SNOW := 0xf2dc
const I_SMOG := 0xf75f
const I_HOUSE := 0xf015
const I_CLIPBOARD := 0xf46c
const I_WRENCH := 0xf7d9
const I_FLAG := 0xf11e
const I_FILE := 0xf15c
const I_STOP := 0xf04d
const I_LOCK := 0xf023
const I_PEN := 0xf304
const I_UNDO := 0xf2ea
const I_LIST_CHECK := 0xf0ae
const I_ARROW_LEFT := 0xf060
const I_IMPORT := 0xf56f
const I_XCIRCLE := 0xf057
const I_SPINNER := 0xf110
const I_STOPWATCH := 0xf2f2
const I_SLIDERS := 0xf1de
const I_ROAD := 0xf018
const I_CLOUD := 0xf743
const I_CUBE := 0xf1b2
const ICONS := {"road": I_ROAD, "burst": I_BURST, "car": I_CAR, "cloud": I_CLOUD, "stopwatch": I_STOPWATCH}

static func ic(code: int) -> String:
	return String.chr(code)

static func hexc(c: Color) -> String:
	return "#" + c.to_html(false)

static func panel_style(bg: Color, radius := 12, pad := 12, border := Color(0, 0, 0, 0)) -> StyleBoxFlat:
	var s := StyleBoxFlat.new()
	s.bg_color = bg
	s.set_corner_radius_all(radius)
	s.content_margin_left = pad
	s.content_margin_right = pad
	s.content_margin_top = pad * 0.7
	s.content_margin_bottom = pad * 0.7
	if border.a > 0:
		s.border_color = border
		s.set_border_width_all(1)
	return s

static func make_theme() -> Theme:
	var th := Theme.new()
	th.default_font_size = 15
	for cls in ["Label", "Button", "CheckBox", "OptionButton", "LineEdit", "PopupMenu", "TooltipLabel"]:
		th.set_color("font_color", cls, TEXT)
	th.set_color("font_hover_color", "Button", TEXT)
	th.set_color("font_disabled_color", "Button", Color(MUTED, 0.6))
	th.set_color("font_pressed_color", "Button", BG0)
	th.set_color("font_hover_pressed_color", "Button", BG0)
	th.set_stylebox("panel", "PanelContainer", panel_style(BG0))
	th.set_stylebox("normal", "Button", panel_style(BG2, 8, 10))
	th.set_stylebox("hover", "Button", panel_style(BG3, 8, 10))
	th.set_stylebox("pressed", "Button", panel_style(Color(ACCENT, 0.85), 8, 10))
	th.set_stylebox("hover_pressed", "Button", panel_style(ACCENT, 8, 10))
	th.set_stylebox("disabled", "Button", panel_style(Color(BG2, 0.5), 8, 10))
	th.set_stylebox("focus", "Button", StyleBoxEmpty.new())
	# 체크박스: 배경 없이 글자 + 기본 아이콘
	for st in ["normal", "hover", "pressed", "hover_pressed", "disabled", "focus"]:
		var e := StyleBoxEmpty.new()
		e.content_margin_top = 3
		e.content_margin_bottom = 3
		th.set_stylebox(st, "CheckBox", e)
	th.set_icon("checked", "CheckBox", _check_icon(true, false))
	th.set_icon("unchecked", "CheckBox", _check_icon(false, false))
	th.set_icon("checked_disabled", "CheckBox", _check_icon(true, true))
	th.set_icon("unchecked_disabled", "CheckBox", _check_icon(false, true))
	th.set_constant("h_separation", "CheckBox", 8)
	th.set_color("font_hover_color", "CheckBox", TEXT)
	th.set_color("font_pressed_color", "CheckBox", TEXT)
	th.set_color("font_hover_pressed_color", "CheckBox", TEXT)
	th.set_color("font_disabled_color", "CheckBox", Color(MUTED, 0.6))
	# 선택 상자
	th.set_stylebox("normal", "OptionButton", panel_style(BG2, 8, 10))
	th.set_stylebox("hover", "OptionButton", panel_style(BG3, 8, 10))
	th.set_stylebox("pressed", "OptionButton", panel_style(BG3, 8, 10))
	th.set_stylebox("focus", "OptionButton", StyleBoxEmpty.new())
	th.set_stylebox("disabled", "OptionButton", panel_style(Color(BG2, 0.5), 8, 10))
	th.set_color("font_hover_color", "OptionButton", TEXT)
	th.set_color("font_pressed_color", "OptionButton", TEXT)
	th.set_stylebox("panel", "PopupMenu", panel_style(BG1, 8, 6, BORDER))
	th.set_stylebox("hover", "PopupMenu", panel_style(Color(ACCENT, 0.25), 6, 6))
	th.set_color("font_hover_color", "PopupMenu", TEXT)
	# 입력 칸
	var le := panel_style(BG2, 8, 8)
	th.set_stylebox("normal", "LineEdit", le)
	th.set_stylebox("focus", "LineEdit", panel_style(BG2, 8, 8, ACCENT))
	th.set_stylebox("read_only", "LineEdit", panel_style(Color(BG2, 0.5), 8, 8))
	th.set_color("caret_color", "LineEdit", ACCENT)
	th.set_color("selection_color", "LineEdit", Color(ACCENT, 0.35))
	# 슬라이더
	var track := StyleBoxFlat.new()
	track.bg_color = BG3
	track.set_corner_radius_all(3)
	track.content_margin_top = 3
	track.content_margin_bottom = 3
	var fill := track.duplicate() as StyleBoxFlat
	fill.bg_color = Color(ACCENT, 0.8)
	th.set_stylebox("slider", "HSlider", track)
	th.set_stylebox("grabber_area", "HSlider", fill)
	th.set_stylebox("grabber_area_highlight", "HSlider", fill)
	th.set_icon("grabber", "HSlider", _dot(14, TEXT))
	th.set_icon("grabber_highlight", "HSlider", _dot(14, Color.WHITE))
	th.set_icon("grabber_disabled", "HSlider", _dot(14, MUTED))
	# 스크롤 막대
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0, 0, 0, 0)
	sb.content_margin_left = 4
	var gr := StyleBoxFlat.new()
	gr.bg_color = Color(MUTED, 0.35)
	gr.set_corner_radius_all(4)
	gr.content_margin_left = 4
	var grh := gr.duplicate() as StyleBoxFlat
	grh.bg_color = Color(MUTED, 0.6)
	th.set_stylebox("scroll", "VScrollBar", sb)
	th.set_stylebox("grabber", "VScrollBar", gr)
	th.set_stylebox("grabber_highlight", "VScrollBar", grh)
	th.set_stylebox("grabber_pressed", "VScrollBar", grh)
	# 툴팁
	th.set_stylebox("panel", "TooltipPanel", panel_style(BG1, 6, 8, BORDER))
	th.set_color("font_color", "TooltipLabel", TEXT)
	th.set_font_size("font_size", "TooltipLabel", 13)
	th.set_stylebox("separator", "HSeparator", _line(BORDER))
	th.set_constant("separation", "HSeparator", 8)
	return th

## 체크박스 그림 (18px): 둥근 사각형, 켜짐 = 파란 채움 + 체크 표시
static func _check_icon(on: bool, disabled: bool) -> ImageTexture:
	var d := 18
	var img := Image.create(d, d, false, Image.FORMAT_RGBA8)
	var a := 0.45 if disabled else 1.0
	var fill := Color(ACCENT, a) if on else Color(BG2, a)
	var edge := Color(ACCENT, a) if on else Color(MUTED, 0.8 * a)
	var p0 := Vector2(4.5, 9.5)
	var p1 := Vector2(7.5, 12.5)
	var p2 := Vector2(13.5, 5.5)
	for y in range(d):
		for x in range(d):
			var p := Vector2(x + 0.5, y + 0.5)
			var q := (p - Vector2(9, 9)).abs() - Vector2(5.5, 5.5)       # 둥근 사각형 거리 (반지름 3)
			var sd := Vector2(maxf(q.x, 0), maxf(q.y, 0)).length() + minf(maxf(q.x, q.y), 0.0) - 3.0
			var c := Color(0, 0, 0, 0)
			if sd < 0.5:
				c = fill
				c.a *= clampf(0.5 - sd, 0, 1)
				if not on and sd > -1.5:
					c = edge
					c.a *= clampf(0.5 - sd, 0, 1)
			if on:
				var dl := minf(Geometry2D.get_closest_point_to_segment(p, p0, p1).distance_to(p),
					Geometry2D.get_closest_point_to_segment(p, p1, p2).distance_to(p))
				var w := clampf(1.9 - dl, 0, 1)
				if w > 0:
					c = c.lerp(Color(BG0, a), w)
					c.a = maxf(c.a, w * a)
			img.set_pixel(x, y, c)
	return ImageTexture.create_from_image(img)

static func _line(c: Color) -> StyleBoxLine:
	var s := StyleBoxLine.new()
	s.color = c
	s.thickness = 1
	return s

static func _dot(d: int, c: Color) -> ImageTexture:
	var img := Image.create(d, d, false, Image.FORMAT_RGBA8)
	var r := d / 2.0
	for y in range(d):
		for x in range(d):
			var dist := Vector2(x + 0.5 - r, y + 0.5 - r).length()
			img.set_pixel(x, y, Color(c, clampf(r - dist, 0.0, 1.0)))
	return ImageTexture.create_from_image(img)

static func btn(text: String, tip := "", toggle := false) -> Button:
	var b := Button.new()
	b.text = text
	b.tooltip_text = tip
	b.toggle_mode = toggle
	b.focus_mode = Control.FOCUS_NONE
	return b

static func primary(b: Button) -> Button:
	b.add_theme_stylebox_override("normal", panel_style(Color(ACCENT, 0.85), 8, 10))
	b.add_theme_stylebox_override("hover", panel_style(ACCENT, 8, 10))
	b.add_theme_color_override("font_color", BG0)
	b.add_theme_color_override("font_hover_color", BG0)
	return b

static func chip(text: String, col: Color) -> Label:
	var l := Label.new()
	l.text = text
	l.add_theme_color_override("font_color", col)
	var s := panel_style(Color(col, 0.16), 999, 8)
	s.content_margin_top = 3
	s.content_margin_bottom = 3
	l.add_theme_stylebox_override("normal", s)
	return l

static func set_chip_color(l: Label, col: Color) -> void:
	l.add_theme_color_override("font_color", col)
	var s := panel_style(Color(col, 0.16), 999, 8)
	s.content_margin_top = 3
	s.content_margin_bottom = 3
	l.add_theme_stylebox_override("normal", s)

static func label(text: String, col := TEXT, size := 0) -> Label:
	var l := Label.new()
	l.text = text
	l.add_theme_color_override("font_color", col)
	if size > 0:
		l.add_theme_font_size_override("font_size", size)
	return l

static func section(parent: Control, icon: int, title: String, col: Color) -> Label:
	var l := label(ic(icon) + "  " + title, col, 14)
	parent.add_child(l)
	return l

## 카드: 테두리 있는 어두운 상자 + 세로 묶음 (반환 = 안쪽 VBox)
static func card(parent: Control) -> VBoxContainer:
	var pc := PanelContainer.new()
	pc.add_theme_stylebox_override("panel", panel_style(BG1, 10, 12, BORDER))
	parent.add_child(pc)
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", 6)
	pc.add_child(v)
	return v

## 패널이 마우스 버튼·휠 입력을 먹게 한다 (Godot 은 휠을 STOP 컨트롤 너머로 넘기므로 3D 시점이 같이 움직이는 것 방지)
static func eat_mouse(ctrl: Control) -> void:
	ctrl.mouse_filter = Control.MOUSE_FILTER_STOP
	ctrl.gui_input.connect(func(e): if e is InputEventMouseButton: ctrl.accept_event())
