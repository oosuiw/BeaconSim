"""Dear ImGui (imgui_bundle) ↔ moderngl 렌더러 + GLFW 입력.

imgui 1.92+의 동적 폰트 텍스처(RendererHasTextures)를 지원한다.
→ 한글처럼 글리프가 많은 폰트도 필요한 글자만 그때그때 아틀라스에 올라간다.
"""
import ctypes

import glfw
import moderngl
import numpy as np
from imgui_bundle import imgui

_VS = """
#version 330
uniform mat4 proj;
in vec2 pos;
in vec2 uv;
in vec4 col;
out vec2 f_uv;
out vec4 f_col;
void main() { f_uv = uv; f_col = col; gl_Position = proj * vec4(pos, 0, 1); }
"""
_FS = """
#version 330
uniform sampler2D tex;
in vec2 f_uv;
in vec4 f_col;
out vec4 out_col;
void main() { out_col = f_col * texture(tex, f_uv); }
"""


class ImguiRenderer:
    def __init__(self, ctx: moderngl.Context):
        self.ctx = ctx
        self.io = imgui.get_io()
        self.io.backend_flags |= imgui.BackendFlags_.renderer_has_textures
        self.prog = ctx.program(vertex_shader=_VS, fragment_shader=_FS)
        self.prog['tex'].value = 0
        self.vbo = ctx.buffer(reserve=imgui.VERTEX_SIZE * 65536, dynamic=True)
        self.ibo = ctx.buffer(reserve=imgui.INDEX_SIZE * 65536, dynamic=True)
        self.vao = ctx.vertex_array(self.prog, [(self.vbo, '2f 2f 4f1', 'pos', 'uv', 'col')],
                                    index_buffer=self.ibo, index_element_size=imgui.INDEX_SIZE)
        self.textures = {}

    # ---- 동적 텍스처 (폰트 아틀라스 등)
    def _update_textures(self):
        for tex in imgui.get_platform_io().textures:
            if tex.status == imgui.ImTextureStatus.ok:
                continue
            if tex.status == imgui.ImTextureStatus.want_create:
                t = self.ctx.texture((tex.width, tex.height), 4, data=tex.get_pixels_array().tobytes())
                t.filter = (moderngl.LINEAR, moderngl.LINEAR)
                self.textures[t.glo] = t
                tex.set_tex_id(t.glo)
                tex.status = imgui.ImTextureStatus.ok
            elif tex.status == imgui.ImTextureStatus.want_updates:
                t = self.textures[tex.tex_id]
                t.write(tex.get_pixels_array().tobytes())         # 갱신은 드물어서 전체 재업로드
                tex.status = imgui.ImTextureStatus.ok
            elif tex.status == imgui.ImTextureStatus.want_destroy:
                t = self.textures.pop(tex.tex_id, None)
                if t is not None:
                    t.release()
                tex.set_tex_id(0)
                tex.status = imgui.ImTextureStatus.destroyed

    def render(self, draw_data):
        self._update_textures()
        io = self.io
        dw, dh = io.display_size
        fbw, fbh = int(dw * io.display_framebuffer_scale[0]), int(dh * io.display_framebuffer_scale[1])
        if fbw == 0 or fbh == 0:
            return
        draw_data.scale_clip_rects(io.display_framebuffer_scale)
        L, T = draw_data.display_pos
        R, B = L + dw, T + dh
        proj = np.array([[2 / (R - L), 0, 0, 0], [0, 2 / (T - B), 0, 0], [0, 0, -1, 0],
                         [(R + L) / (L - R), (T + B) / (B - T), 0, 1]], dtype='f4')
        self.prog['proj'].write(proj.tobytes())
        ctx = self.ctx
        ctx.enable_only(moderngl.BLEND)
        ctx.blend_equation = moderngl.FUNC_ADD
        ctx.blend_func = moderngl.SRC_ALPHA, moderngl.ONE_MINUS_SRC_ALPHA
        ctx.viewport = (0, 0, fbw, fbh)
        for cl in draw_data.cmd_lists:
            vsz = cl.vtx_buffer.size() * imgui.VERTEX_SIZE
            isz = cl.idx_buffer.size() * imgui.INDEX_SIZE
            if vsz > self.vbo.size:
                self.vbo.orphan(vsz * 2)
            if isz > self.ibo.size:
                self.ibo.orphan(isz * 2)
            self.vbo.write((ctypes.c_byte * vsz).from_address(cl.vtx_buffer.data_address()))
            self.ibo.write((ctypes.c_byte * isz).from_address(cl.idx_buffer.data_address()))
            for cmd in cl.cmd_buffer:
                t = self.textures.get(cmd.tex_ref.get_tex_id())
                if t is not None:
                    t.use(0)
                x, y, z, w = cmd.clip_rect
                ctx.scissor = (int(x), int(fbh - w), int(z - x), int(w - y))
                self.vao.render(moderngl.TRIANGLES, vertices=cmd.elem_count, first=cmd.idx_offset)
        ctx.scissor = None
        ctx.disable(moderngl.BLEND)


_KEYS = {
    glfw.KEY_TAB: 'tab', glfw.KEY_BACKSPACE: 'backspace', glfw.KEY_ENTER: 'enter', glfw.KEY_ESCAPE: 'escape',
    glfw.KEY_DELETE: 'delete', glfw.KEY_SPACE: 'space', glfw.KEY_LEFT: 'left_arrow', glfw.KEY_RIGHT: 'right_arrow',
    glfw.KEY_UP: 'up_arrow', glfw.KEY_DOWN: 'down_arrow', glfw.KEY_HOME: 'home', glfw.KEY_END: 'end',
    glfw.KEY_PAGE_UP: 'page_up', glfw.KEY_PAGE_DOWN: 'page_down', glfw.KEY_KP_ENTER: 'keypad_enter',
    glfw.KEY_A: 'a', glfw.KEY_C: 'c', glfw.KEY_V: 'v', glfw.KEY_X: 'x', glfw.KEY_Z: 'z', glfw.KEY_Y: 'y',
    glfw.KEY_LEFT_CONTROL: 'left_ctrl', glfw.KEY_RIGHT_CONTROL: 'right_ctrl',
    glfw.KEY_LEFT_SHIFT: 'left_shift', glfw.KEY_RIGHT_SHIFT: 'right_shift',
    glfw.KEY_LEFT_ALT: 'left_alt', glfw.KEY_RIGHT_ALT: 'right_alt',
}


class GlfwInput:
    """GLFW 이벤트 → imgui IO. 3D 뷰 조작용 콜백(on_drag/on_scroll/on_key)도 넘겨준다."""

    def __init__(self, win, on_drag=None, on_scroll=None, on_key=None):
        self.win = win
        self.io = imgui.get_io()
        self.on_drag, self.on_scroll, self.on_key = on_drag, on_scroll, on_key
        self.keymap = {k: getattr(imgui.Key, v) for k, v in _KEYS.items()}
        self.last_pos = None
        self.buttons = [False, False, False]
        glfw.set_cursor_pos_callback(win, self._pos)
        glfw.set_mouse_button_callback(win, self._button)
        glfw.set_scroll_callback(win, self._scroll)
        glfw.set_key_callback(win, self._key)
        glfw.set_char_callback(win, self._char)
        pio = imgui.get_platform_io()
        pio.platform_get_clipboard_text_fn = lambda _c: (glfw.get_clipboard_string(win) or b'').decode()
        pio.platform_set_clipboard_text_fn = lambda _c, s: glfw.set_clipboard_string(win, s)

    def new_frame(self, dt):
        io = self.io
        ww, wh = glfw.get_window_size(self.win)
        fw, fh = glfw.get_framebuffer_size(self.win)
        io.display_size = imgui.ImVec2(ww, wh)
        if ww > 0 and wh > 0:
            io.display_framebuffer_scale = imgui.ImVec2(fw / ww, fh / wh)
        io.delta_time = max(dt, 1e-4)

    def _pos(self, win, x, y):
        self.io.add_mouse_pos_event(x, y)
        if self.last_pos is not None and any(self.buttons) and not self.io.want_capture_mouse and self.on_drag:
            self.on_drag(x - self.last_pos[0], y - self.last_pos[1], list(self.buttons))
        self.last_pos = (x, y)

    def _button(self, win, button, action, mods):
        if button < 3:
            down = action == glfw.PRESS
            self.io.add_mouse_button_event(button, down)
            if not down or not self.io.want_capture_mouse:
                self.buttons[button] = down

    def _scroll(self, win, dx, dy):
        self.io.add_mouse_wheel_event(dx, dy)
        if not self.io.want_capture_mouse and self.on_scroll:
            self.on_scroll(dy)

    def _key(self, win, key, scancode, action, mods):
        down = action != glfw.RELEASE
        k = self.keymap.get(key)
        if k is not None:
            self.io.add_key_event(k, down)
        self.io.add_key_event(imgui.Key.mod_ctrl, bool(mods & glfw.MOD_CONTROL))
        self.io.add_key_event(imgui.Key.mod_shift, bool(mods & glfw.MOD_SHIFT))
        self.io.add_key_event(imgui.Key.mod_alt, bool(mods & glfw.MOD_ALT))
        if action == glfw.PRESS and not self.io.want_capture_keyboard and self.on_key:
            self.on_key(key)

    def _char(self, win, cp):
        self.io.add_input_character(cp)
