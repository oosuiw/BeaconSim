"""moderngl 렌더러 (GUI 뷰어, 카메라 센서 공용).

SceneRenderer  : 정적 씬 + 로봇 + 차량(등화 포함), 환경(시간대/날씨) 조명·대기 산란·가로등
CameraRenderer : Arducam 모델 — 오버스캔 핀홀 렌더 → 왜곡 remap → 환경 후처리(노이즈/비/눈)
OverlayRenderer: 점(센서 점군)·선(SCC 표시, GNSS, 레이더 객체 등) 동적 오버레이
"""
import cv2
import moderngl
import numpy as np

from . import scene
from .common import ROBOT_PART_COLORS

MAX_LIGHTS = 24

VS = """
#version 330
uniform mat4 mvp;
uniform mat4 model;
in vec3 in_pos;
in vec3 in_nrm;
in vec3 in_col;
in float in_light;
out vec3 v_nrm;
out vec3 v_col;
out vec3 v_world;
out float v_light;
void main() {
    vec4 w = model * vec4(in_pos, 1.0);
    v_world = w.xyz;
    v_nrm = mat3(model) * in_nrm;
    v_col = in_col;
    v_light = in_light;
    gl_Position = mvp * vec4(in_pos, 1.0);
}
"""

FS = """
#version 330
uniform vec3 sun_dir;
uniform vec3 cam_pos;
uniform vec3 fog_color;
uniform float sun;          // 직사광
uniform float ambient;      // 주변광
uniform float obj_light;
uniform float fog_dist;
uniform int lamps_on;       // 차량 등화 발광
uniform int n_lights;
uniform vec3 lights[24];    // 가로등 위치
in vec3 v_nrm;
in vec3 v_col;
in vec3 v_world;
in float v_light;
out vec4 f_color;
void main() {
    if (v_light > 1.5) {                            // 등화 (발광체)
        if (lamps_on == 1) { f_color = vec4(v_col * 1.6, 1.0); return; }
    }
    vec3 n = normalize(v_nrm);
    float diff = abs(dot(n, sun_dir));              // 양면 조명
    float outside = ambient + sun * diff;
    for (int i = 0; i < n_lights; ++i) {            // 가로등: 수평 거리 가우시안 감쇠
        vec3 d = lights[i] - v_world;
        float r2 = dot(d.xy, d.xy);
        outside += 0.55 * exp(-r2 / 260.0) * (0.4 + 0.6 * max(0.0, normalize(d).z));
    }
    float k = min(v_light, 1.0) * obj_light;        // 터널 내부: 정점별 조명 계수
    float tunnel = 0.30 + 0.15 * abs(n.z);          // 터널 조명 (주야 동일)
    float light = mix(tunnel, outside, clamp((k - 0.4) / 0.6, 0.0, 1.0));
    vec3 c = v_col * light;
    float dist = length(v_world - cam_pos);
    float fog = 1.0 - exp(-dist / fog_dist);
    f_color = vec4(mix(c, fog_color, fog), 1.0);
}
"""

SKY = (0.62, 0.74, 0.88)


def flat_vertices(v, f, color, light=None, emissive=False):
    """인덱스 메시 → 면 법선을 가진 비인덱스 정점 (pos3, nrm3, col3, light1). emissive면 light=2."""
    tri = v[f].astype(np.float32)
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    nrm = np.repeat(n[:, None, :], 3, axis=1)
    col = np.broadcast_to(np.asarray(color, np.float32), tri.shape)
    if emissive:
        lt = np.full(tri.shape[:2] + (1,), 2.0, np.float32)
    elif light is None:
        lt = np.ones(tri.shape[:2] + (1,), np.float32)
    else:
        lt = light[f][..., None].astype(np.float32)
    return np.concatenate([tri, nrm, col, lt], axis=2).reshape(-1, 10).astype(np.float32)


class SceneRenderer:
    def __init__(self, ctx: moderngl.Context, model, robot_parts=None, exclude_robot=('camera',), fog_dist=900.0,
                 parts=None, env=None):
        self.ctx = ctx
        self.prog = ctx.program(vertex_shader=VS, fragment_shader=FS)
        sun = np.array([0.4, -0.3, 0.85])
        self.prog['sun_dir'].value = tuple(sun / np.linalg.norm(sun))
        self.base_fog = fog_dist
        self.robot = []
        if robot_parts:
            self.robot = [self._vao(flat_vertices(v, f, ROBOT_PART_COLORS.get(name, (0.6, 0.6, 0.6))))
                          for name, (v, f) in robot_parts.items() if name not in exclude_robot]
        self.static, self.veh_vao = [], {}
        self.lights = np.zeros((0, 3))
        self.set_model(model, parts)
        self.set_env(env)

    def _vao(self, data):
        vbo = self.ctx.buffer(data.tobytes())
        return self.ctx.vertex_array(self.prog, [(vbo, '3f 3f 3f 1f', 'in_pos', 'in_nrm', 'in_col', 'in_light')]), vbo

    def set_model(self, model, parts=None):
        for vao, vbo in self.static:
            vao.release()
            vbo.release()
        self.model = model
        parts = parts if parts is not None else scene.build_static(model)
        self.static, self.static_names = [], []
        for p in parts:
            light = scene.tunnel_light(model, p.sth) if p.sth is not None else None
            self.static.append(self._vao(flat_vertices(p.v, p.f, p.color, light)))
            self.static_names.append(p.name)
        self.lights = scene.street_lights(model)

    def set_env(self, env):
        """env: environment.Env 또는 None(맑은 낮)."""
        self.env = env
        if env is None:
            self.sky, sun, amb, fog, fogc, lamps = SKY, 0.65, 0.45, self.base_fog, SKY, 0
        else:
            self.sky, sun, amb, fog, fogc = env.sky, env.sun, env.ambient, min(env.fog_dist, self.base_fog), env.fog_color
            lamps = int(env.lamps)
        self.prog['sun'].value = float(sun)
        self.prog['ambient'].value = float(amb)
        self.prog['fog_dist'].value = float(fog)
        self.prog['fog_color'].value = tuple(float(c) for c in fogc)
        self.prog['lamps_on'].value = lamps
        self.street_on = bool(env is not None and env.street_lights)

    def vehicle_vaos(self, spec):
        """spec: traffic.VehicleSpec → [vao] (외형별 캐시: 차체 + 등화)."""
        if spec.key not in self.veh_vao:
            v, f = scene.vehicle_mesh(spec.kind, spec.length, spec.width, spec.height)
            lst = [self._vao(flat_vertices(v, f, spec.color))[0]]
            for lv, lf, lc in scene.vehicle_lamps(spec.kind, spec.length, spec.width, spec.height):
                lst.append(self._vao(flat_vertices(lv, lf, lc, emissive=True))[0])
            self.veh_vao[spec.key] = lst
        return self.veh_vao[spec.key]

    def _draw(self, vao, model_mat, VP, light=1.0):
        self.prog['model'].write(model_mat.T.astype('f4').tobytes())
        self.prog['mvp'].write((VP @ model_mat).T.astype('f4').tobytes())
        self.prog['obj_light'].value = float(light)
        vao.render(moderngl.TRIANGLES)

    def _light_at(self, T):
        s, t = self.model.project(T[0, 3], T[1, 3])
        return float(scene.tunnel_light(self.model, np.array([[s, t, 1.0]]))[0])

    def draw(self, VP, cam_pos, T_wb=None, hidden=(), vehicles=()):
        """vehicles: [(VehicleSpec, T_world)]."""
        cam_pos = np.asarray(cam_pos, float)
        self.prog['cam_pos'].value = tuple(float(c) for c in cam_pos)
        if self.street_on and len(self.lights):
            d = np.linalg.norm(self.lights[:, :2] - cam_pos[:2], axis=1)
            near = self.lights[np.argsort(d)[:MAX_LIGHTS]]
            arr = np.zeros((MAX_LIGHTS, 3), 'f4')
            arr[:len(near)] = near
            self.prog['lights'].write(arr.tobytes())
            self.prog['n_lights'].value = len(near)
        else:
            self.prog['n_lights'].value = 0
        I = np.eye(4)
        for (vao, _), name in zip(self.static, self.static_names):
            if name not in hidden:
                self._draw(vao, I, VP)
        if T_wb is not None:
            lt = self._light_at(T_wb)
            for vao, _ in self.robot:
                self._draw(vao, T_wb, VP, lt)
        for spec, T in vehicles:
            if T[2, 3] < -100:            # 비활성 슬롯
                continue
            lt = self._light_at(T)
            for vao in self.vehicle_vaos(spec):
                self._draw(vao, T, VP, lt)


# ---------------------------------------------------------------- 카메라 센서

def gl_projection(fx, fy, cx, cy, w, h, near, far):
    """OpenCV 내부 파라미터 → OpenGL 투영 (광학 좌표 x우 y하 z전방을 GL 카메라로 바꾼 뒤 적용)."""
    P = np.zeros((4, 4))
    P[0, 0] = 2 * fx / w
    P[1, 1] = 2 * fy / h
    P[0, 2] = 1 - 2 * cx / w
    P[1, 2] = 2 * cy / h - 1
    P[2, 2] = -(far + near) / (far - near)
    P[2, 3] = -2 * far * near / (far - near)
    P[3, 2] = -1
    return P


OPTICAL_TO_GL = np.diag([1.0, -1.0, -1.0, 1.0])


class CameraRenderer:
    """Arducam B0495 모델: 오버스캔 핀홀 렌더(4x MSAA) → plumb_bob 왜곡 remap → 환경 후처리."""

    def __init__(self, ctx, cam_cfg, seed=0):
        self.ctx = ctx
        self.W, self.H = cam_cfg['width'], cam_cfg['height']
        self.K = np.array(cam_cfg['K'], float).reshape(3, 3)
        self.D = np.array(cam_cfg['D'], float)
        fx, fy = self.K[0, 0], self.K[1, 1]
        uu, vv = np.meshgrid(np.arange(self.W, dtype=np.float64), np.arange(self.H, dtype=np.float64))
        pts = np.stack([uu.ravel(), vv.ravel()], -1)[:, None, :]
        und = cv2.undistortPoints(pts, self.K, self.D).reshape(self.H, self.W, 2)
        mx, my = np.abs(und[..., 0]).max(), np.abs(und[..., 1]).max()
        self.Wr, self.Hr = int(2 * np.ceil(fx * mx + 4)), int(2 * np.ceil(fy * my + 4))
        cxr, cyr = self.Wr / 2.0, self.Hr / 2.0
        self.map_x = (fx * und[..., 0] + cxr).astype(np.float32)
        self.map_y = (fy * und[..., 1] + cyr).astype(np.float32)
        self.P = gl_projection(fx, fy, cxr, cyr, self.Wr, self.Hr, cam_cfg['near'], cam_cfg['far'])
        self.hfov = 2 * np.degrees(np.arctan(und[self.H // 2, -1, 0]))
        self.vfov = 2 * np.degrees(np.arctan(und[-1, self.W // 2, 1]))
        self.fbo_ms = ctx.framebuffer(color_attachments=[ctx.renderbuffer((self.Wr, self.Hr), 3, samples=4)],
                                      depth_attachment=ctx.depth_renderbuffer((self.Wr, self.Hr), samples=4))
        self.fbo = ctx.framebuffer(color_attachments=[ctx.renderbuffer((self.Wr, self.Hr), 3)])
        self.rng = np.random.default_rng(seed)

    def render(self, scene_r: SceneRenderer, T_w_opt, T_wb, vehicles, env=None):
        """→ RGB uint8 (H, W, 3)."""
        VP = self.P @ OPTICAL_TO_GL @ np.linalg.inv(T_w_opt)
        self.fbo_ms.use()
        self.ctx.viewport = (0, 0, self.Wr, self.Hr)
        self.ctx.clear(*scene_r.sky, 1.0)
        self.ctx.enable(moderngl.DEPTH_TEST)
        scene_r.draw(VP, T_w_opt[:3, 3], T_wb, vehicles=vehicles)
        self.ctx.copy_framebuffer(self.fbo, self.fbo_ms)
        raw = np.frombuffer(self.fbo.read(components=3, alignment=1), np.uint8).reshape(self.Hr, self.Wr, 3)
        img = cv2.remap(raw[::-1], self.map_x, self.map_y, cv2.INTER_LINEAR)
        return self._post(img, env)

    def _bank(self, env):
        """환경별 후처리 레이어 미리 생성 (노이즈 4장, 빗줄기/눈 4장) → 프레임마다 돌려 씀."""
        key = (env.weather, round(env.intensity, 2), env.time_of_day)
        if getattr(self, '_bank_key', None) == key:
            return self._bank_data
        rng = np.random.default_rng(1)
        H, W = self.H, self.W
        noise = [rng.normal(0, env.noise, (H, W, 3)).astype(np.int16) for _ in range(4)]
        layers = []
        for _ in range(4):
            layer = np.zeros((H, W), np.uint8)
            if env.weather == 'rain':
                n = int(300 + 1500 * env.intensity)
                for xi, yi, li in zip(rng.integers(0, W, n), rng.integers(0, H, n), rng.integers(15, 60, n)):
                    cv2.line(layer, (int(xi), int(yi)), (int(xi + li * 0.15), int(yi + li)), 255, 1)
            elif env.weather == 'snow':
                n = int(400 + 2500 * env.intensity)
                for xi, yi, r in zip(rng.integers(0, W, n), rng.integers(0, H, n), rng.integers(1, 4, n)):
                    cv2.circle(layer, (int(xi), int(yi)), int(r), 255, -1)
            layers.append(layer)
        self._bank_key, self._bank_data = key, (noise, layers)
        self._k = 0
        return self._bank_data

    def _post(self, img, env):
        if env is None:
            return img
        noise, layers = self._bank(env)
        self._k = (self._k + 1) % 4
        out = img
        if env.weather in ('rain', 'fog'):
            out = cv2.GaussianBlur(out, (0, 0), 0.6 + 1.2 * env.intensity)
        out = out.astype(np.int16) + noise[self._k]
        L = layers[self._k]
        if env.weather == 'rain':
            out += (L[..., None].astype(np.int16) * (60 if env.time_of_day == 'day' else 30)) // 255
        elif env.weather == 'snow':
            m = (L > 0)[..., None]
            out = np.where(m, (out * 2 + 235 * 8) // 10, out)
        return np.clip(out, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------- 오버레이

_OV_VS = """
#version 330
uniform mat4 mvp;
uniform float psize;
in vec3 in_pos;
in vec4 in_col;
out vec4 v_col;
void main() { v_col = in_col; gl_Position = mvp * vec4(in_pos, 1.0); gl_PointSize = psize; }
"""
_OV_FS = """
#version 330
in vec4 v_col;
out vec4 f;
void main() { f = v_col; }
"""


class OverlayRenderer:
    """동적 점/선 그리기 (버퍼 재사용)."""

    def __init__(self, ctx):
        self.ctx = ctx
        self.prog = ctx.program(vertex_shader=_OV_VS, fragment_shader=_OV_FS)
        self.bufs = {}

    def draw(self, key, pts, cols, VP, mode=moderngl.POINTS, size=2.0, blend=False):
        """pts (N,3), cols (N,4) 또는 (4,). blend=True면 알파 블렌딩 (반투명 면)."""
        n = len(pts)
        if n == 0:
            return
        cols = np.broadcast_to(np.asarray(cols, np.float32), (n, 4))
        data = np.concatenate([np.asarray(pts, np.float32), cols], 1).astype('f4').tobytes()
        if key not in self.bufs or self.bufs[key][0].size < len(data):
            if key in self.bufs:
                self.bufs[key][1].release()
                self.bufs[key][0].release()
            vbo = self.ctx.buffer(reserve=max(len(data) * 2, 1024), dynamic=True)
            vao = self.ctx.vertex_array(self.prog, [(vbo, '3f 4f', 'in_pos', 'in_col')])
            self.bufs[key] = (vbo, vao)
        vbo, vao = self.bufs[key]
        vbo.write(data)
        self.prog['mvp'].write(VP.T.astype('f4').tobytes())
        self.prog['psize'].value = float(size)
        if blend:
            self.ctx.enable(moderngl.BLEND)
            self.ctx.blend_func = moderngl.SRC_ALPHA, moderngl.ONE_MINUS_SRC_ALPHA
        vao.render(mode, vertices=n)
        if blend:
            self.ctx.disable(moderngl.BLEND)
