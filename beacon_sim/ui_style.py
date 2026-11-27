"""BeaconSim UI 디자인 시스템: 팔레트, 폰트(Pretendard + Font Awesome 6), 위젯 헬퍼."""
import os

from imgui_bundle import icons_fontawesome_6 as FA  # noqa: F401  (아이콘 상수 재노출)
from imgui_bundle import imgui

from .common import ROOT

FONT_DIR = os.path.join(ROOT, 'assets', 'fonts')

# 팔레트 (RGBA 0~1)
BG0 = (0.055, 0.067, 0.086, 0.97)      # 창 배경
BG1 = (0.082, 0.098, 0.125, 1.0)       # 카드
BG2 = (0.118, 0.137, 0.170, 1.0)       # 입력 필드
BG3 = (0.160, 0.185, 0.225, 1.0)       # 호버
BORDER = (0.165, 0.190, 0.230, 1.0)
TEXT = (0.902, 0.918, 0.941, 1.0)
MUTED = (0.545, 0.588, 0.647, 1.0)
ACCENT = (0.302, 0.639, 1.0, 1.0)      # 파랑
TEAL = (0.169, 0.831, 0.773, 1.0)
OK = (0.239, 0.863, 0.518, 1.0)
WARN = (1.0, 0.722, 0.302, 1.0)
DANGER = (1.0, 0.353, 0.373, 1.0)
SCC = (0.780, 0.490, 1.0, 1.0)         # 보라
AMBER = (1.0, 0.600, 0.200, 1.0)


def v4(c, a=None):
    return imgui.ImVec4(c[0], c[1], c[2], c[3] if a is None else a)


def u32(c, a=None):
    return imgui.get_color_u32(v4(c, a))


class Fonts:
    body = title = bold = None
    BODY, SMALL, TITLE, BIG, HUGE = 16.0, 13.5, 18.0, 26.0, 34.0


CACHE_DIR = os.path.join(os.path.expanduser('~'), '.cache', 'beaconsim', 'fonts')


def _ui_font(weight):
    """원본 Pretendard에서 PUA(U+E000–F8FF) 글리프만 뺀 사본을 사용자 캐시에 만든다.
    Pretendard에 PUA 글리프가 있어 Font Awesome 아이콘을 가리기 때문. 수정본은 배포하지 않는다(OFL 예약 이름)."""
    src = os.path.join(FONT_DIR, f'Pretendard-{weight}.otf')
    if not os.path.exists(src):
        return None
    dst = os.path.join(CACHE_DIR, f'pretendard-{weight.lower()}-nopua.otf')
    if os.path.exists(dst) and os.path.getmtime(dst) >= os.path.getmtime(src):
        return dst
    try:
        from fontTools import subset
        os.makedirs(CACHE_DIR, exist_ok=True)
        opts = subset.Options()
        opts.layout_features = ['*']
        opts.name_IDs = ['*']
        opts.notdef_outline = True
        fnt = subset.load_font(src, opts)
        sub = subset.Subsetter(opts)
        cmap = fnt.getBestCmap()
        sub.populate(unicodes=[c for c in cmap if not 0xE000 <= c <= 0xF8FF])
        sub.subset(fnt)
        subset.save_font(fnt, dst, opts)
        return dst
    except Exception:          # noqa: BLE001  fontTools 없으면 원본 사용 (일부 아이콘이 가려질 수 있음)
        return src


def load_fonts(scale=1.0):
    """Pretendard Medium/SemiBold/Bold + Font Awesome 병합. 없으면 시스템 한글 폰트로 대체."""
    io = imgui.get_io()
    fa = os.path.join(FONT_DIR, 'Font_Awesome_6_Free-Solid-900.otf')

    def add(path, size):
        f = io.fonts.add_font_from_file_ttf(path, size * scale)
        if os.path.exists(fa):
            c = imgui.ImFontConfig()
            c.merge_mode = True
            c.glyph_offset = imgui.ImVec2(0, 1 * scale)
            io.fonts.add_font_from_file_ttf(fa, size * 0.85 * scale, c)
        return f

    pre = {w: _ui_font(w) for w in ('Medium', 'SemiBold', 'Bold')}
    if all(p and os.path.exists(p) for p in pre.values()):
        Fonts.body = add(pre['Medium'], Fonts.BODY)
        Fonts.title = add(pre['SemiBold'], Fonts.TITLE)
        Fonts.bold = add(pre['Bold'], Fonts.BIG)
        return pre['Medium']
    for p in ['/usr/share/fonts/truetype/nanum/NanumGothic.ttf', '/usr/share/fonts/truetype/unfonts-core/UnDotum.ttf']:
        if os.path.exists(p):
            Fonts.body = Fonts.title = Fonts.bold = add(p, Fonts.BODY)
            return p
    Fonts.body = Fonts.title = Fonts.bold = io.fonts.add_font_default()
    return '(기본 폰트)'


ZOOM = 1.0          # 현재 UI 배율 (창 크기 · 모니터 배율에 따라 set_zoom 으로 바뀜)
_BASE = None
_NO_SCALE = {'alpha', 'disabled_alpha', 'font_scale_main', 'font_scale_dpi', 'font_size_base', 'mouse_cursor_scale',
             'curve_tessellation_max_error', 'circle_tessellation_max_error', 'hover_delay_normal', 'hover_delay_short',
             'hover_stationary_delay', 'log_slider_deadzone', 'table_angled_headers_angle'}


def set_zoom(z):
    """UI 전체 배율: 기본 크기(apply_theme 직후 값)에서 다시 계산하므로 여러 번 불러도 누적되지 않는다."""
    global _BASE, ZOOM
    st = imgui.get_style()
    if _BASE is None:
        _BASE = {}
        for n in dir(st):
            if n.startswith('_') or n in _NO_SCALE or 'align' in n:
                continue
            v = getattr(st, n)
            if isinstance(v, float):
                _BASE[n] = v
            elif isinstance(v, imgui.ImVec2):
                _BASE[n] = (v.x, v.y)
    for n, v in _BASE.items():
        setattr(st, n, imgui.ImVec2(*v) if isinstance(v, tuple) else v)
    st.scale_all_sizes(z)
    st.font_scale_main = z
    ZOOM = z


def px(v):
    return v * ZOOM


def apply_theme(scale=1.0):
    st = imgui.get_style()
    st.window_rounding, st.child_rounding, st.frame_rounding = 10.0, 8.0, 6.0
    st.popup_rounding, st.grab_rounding, st.tab_rounding, st.scrollbar_rounding = 8.0, 6.0, 6.0, 6.0
    st.window_border_size, st.frame_border_size, st.child_border_size, st.popup_border_size = 0.0, 0.0, 0.0, 1.0
    st.window_padding = imgui.ImVec2(14, 12)
    st.frame_padding = imgui.ImVec2(10, 6)
    st.item_spacing = imgui.ImVec2(8, 7)
    st.item_inner_spacing = imgui.ImVec2(6, 4)
    st.scrollbar_size, st.grab_min_size = 9.0, 10.0
    st.separator_text_border_size = 1.0
    C = imgui.Col_
    pal = {
        C.text: TEXT, C.text_disabled: MUTED, C.window_bg: BG0, C.child_bg: (0, 0, 0, 0), C.popup_bg: BG1,
        C.border: BORDER, C.frame_bg: BG2, C.frame_bg_hovered: BG3, C.frame_bg_active: BG3,
        C.title_bg: BG1, C.title_bg_active: BG1, C.title_bg_collapsed: BG1,
        C.scrollbar_bg: (0, 0, 0, 0), C.scrollbar_grab: BG3, C.scrollbar_grab_hovered: (0.22, 0.26, 0.31, 1),
        C.check_mark: ACCENT, C.slider_grab: ACCENT, C.slider_grab_active: (0.5, 0.75, 1, 1),
        C.button: BG2, C.button_hovered: BG3, C.button_active: (0.20, 0.28, 0.38, 1),
        C.header: (0.302, 0.639, 1.0, 0.16), C.header_hovered: (0.302, 0.639, 1.0, 0.26),
        C.header_active: (0.302, 0.639, 1.0, 0.34), C.separator: BORDER,
        C.tab: (0, 0, 0, 0), C.tab_hovered: BG3, C.tab_selected: BG2,
        C.tab_selected_overline: ACCENT, C.resize_grip: (0, 0, 0, 0),
        C.plot_histogram: ACCENT, C.modal_window_dim_bg: (0, 0, 0, 0.55),
    }
    for col, rgba in pal.items():
        st.set_color_(col, v4(rgba))
    if scale > 1:
        st.scale_all_sizes(scale)


# ---------------------------------------------------------------- 위젯

def tip(text):
    if imgui.is_item_hovered(imgui.HoveredFlags_.delay_short):
        imgui.set_tooltip(text)


def section(icon, title, color=ACCENT):
    """카드 안 섹션 제목: 아이콘 + 굵은 글씨 + 얇은 구분선."""
    imgui.push_font(Fonts.title, Fonts.TITLE * 0.85)
    imgui.text_colored(v4(color), f'{icon}  {title}')
    imgui.pop_font()


def muted(text):
    imgui.text_colored(v4(MUTED), text)


def chip(text, color=ACCENT, icon=None, filled=False):
    """알약 모양 상태 표시."""
    label = f'{icon}  {text}' if icon else text
    sz = imgui.calc_text_size(label)
    p = imgui.get_cursor_screen_pos()
    padx, pady = px(9), px(3)
    dl = imgui.get_window_draw_list()
    dl.add_rect_filled(p, imgui.ImVec2(p.x + sz.x + 2 * padx, p.y + sz.y + 2 * pady),
                       u32(color, 0.9 if filled else 0.16), 999.0)
    dl.add_text(imgui.ImVec2(p.x + padx, p.y + pady), u32(BG0 if filled else color), label)
    imgui.dummy(imgui.ImVec2(sz.x + 2 * padx, sz.y + 2 * pady))


def kpi(label, value, color=TEXT, width=None):
    """작은 회색 라벨 + 큰 숫자."""
    imgui.begin_group()
    imgui.push_font(Fonts.body, Fonts.SMALL)
    muted(label)
    imgui.pop_font()
    imgui.push_font(Fonts.bold, Fonts.BIG)
    imgui.text_colored(v4(color), str(value))
    imgui.pop_font()
    imgui.end_group()
    if width:
        imgui.same_line(width)


def begin_card(cid, height=0.0):
    imgui.push_style_color(imgui.Col_.child_bg, v4(BG1))
    imgui.push_style_var(imgui.StyleVar_.window_padding, imgui.ImVec2(px(12), px(10)))
    flags = imgui.ChildFlags_.always_use_window_padding
    if height == 0.0:
        flags |= imgui.ChildFlags_.auto_resize_y
    imgui.begin_child(cid, imgui.ImVec2(0, height), flags)


def end_card():
    imgui.end_child()
    imgui.pop_style_var()
    imgui.pop_style_color()


def toggle_button(icon, label, on, color=ACCENT, size=None):
    """켜짐/꺼짐 아이콘 버튼. 반환: 새 상태."""
    if on:
        imgui.push_style_color(imgui.Col_.button, v4(color, 0.22))
        imgui.push_style_color(imgui.Col_.button_hovered, v4(color, 0.32))
        imgui.push_style_color(imgui.Col_.text, v4(color))
    else:
        imgui.push_style_color(imgui.Col_.button, v4(BG2, 0.6))
        imgui.push_style_color(imgui.Col_.button_hovered, v4(BG3))
        imgui.push_style_color(imgui.Col_.text, v4(MUTED))
    clicked = imgui.button(f'{icon}  {label}' if label else icon, size or imgui.ImVec2(0, 0))
    imgui.pop_style_color(3)
    return (not on) if clicked else on


def primary_button(label, size=None, color=ACCENT):
    imgui.push_style_color(imgui.Col_.button, v4(color, 0.85))
    imgui.push_style_color(imgui.Col_.button_hovered, v4(color, 1.0))
    imgui.push_style_color(imgui.Col_.button_active, v4(color, 0.7))
    imgui.push_style_color(imgui.Col_.text, v4(BG0, 1.0))
    r = imgui.button(label, size or imgui.ImVec2(0, 0))
    imgui.pop_style_color(4)
    return r


def segmented(items, current, width=None):
    """세그먼트 선택 (가로 버튼 묶음). 반환: 선택 인덱스."""
    out = current
    imgui.push_style_var(imgui.StyleVar_.item_spacing, imgui.ImVec2(px(2), 0))
    for i, it in enumerate(items):
        if i:
            imgui.same_line()
        sel = i == current
        imgui.push_style_color(imgui.Col_.button, v4(ACCENT, 0.85) if sel else v4(BG2))
        imgui.push_style_color(imgui.Col_.text, v4(BG0) if sel else v4(MUTED))
        if imgui.button(it, imgui.ImVec2(width or 0, 0)):
            out = i
        imgui.pop_style_color(2)
    imgui.pop_style_var()
    return out


def pill_label(dl, x, y, text, color):
    """3D 이름표: 색 점 + 어두운 알약."""
    sz = imgui.calc_text_size(text)
    w, h = sz.x + px(26), sz.y + px(8)
    x0, y0 = x - w / 2, y - h
    dl.add_rect_filled(imgui.ImVec2(x0, y0), imgui.ImVec2(x0 + w, y0 + h), u32(BG0, 0.82), 999.0)
    dl.add_rect(imgui.ImVec2(x0, y0), imgui.ImVec2(x0 + w, y0 + h), u32(color, 0.55), 999.0, 1.0)
    dl.add_circle_filled(imgui.ImVec2(x0 + px(11), y0 + h / 2), px(4.0), u32(color))
    dl.add_text(imgui.ImVec2(x0 + px(19), y0 + px(4)), u32(TEXT), text)
    dl.add_line(imgui.ImVec2(x, y0 + h), imgui.ImVec2(x, y + px(6)), u32(color, 0.6), px(1.0))
