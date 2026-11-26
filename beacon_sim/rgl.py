"""Robotec GPU Lidar(RGL) core API의 최소 ctypes 래퍼.

헤더: third_party/rgl_src/include/rgl/api/core.h (v0.21.0)
ROS2 확장이 없는 core 빌드를 쓴다. ROS 퍼블리시는 rclpy가 담당한다.
"""
import ctypes as C
import os
import numpy as np

_LIB_PATH = os.environ.get(
    'RGL_LIB',
    os.path.join(os.path.dirname(__file__), '..', 'third_party', 'rgl', 'libRobotecGPULidar.so'))

# rgl_field_t (core.h enum 순서, 컴파일러로 확인한 값)
XYZ_VEC3_F32 = 1
INTENSITY_F32 = 2
IS_HIT_I32 = 4
RAY_IDX_U32 = 6
ENTITY_ID_I32 = 7
DISTANCE_F32 = 8
AZIMUTH_F32 = 9
ELEVATION_F32 = 10
RING_ID_U16 = 11
TIME_STAMP_F64 = 13
ABSOLUTE_VELOCITY_VEC3_F32 = 15
RELATIVE_VELOCITY_VEC3_F32 = 16
RADIAL_SPEED_F32 = 17
NORMAL_VEC3_F32 = 22
INCIDENT_ANGLE_F32 = 23
LASER_RETRO_F32 = 25

FIELD_DTYPE = {
    XYZ_VEC3_F32: (np.float32, 3),
    INTENSITY_F32: (np.float32, 1),
    IS_HIT_I32: (np.int32, 1),
    RAY_IDX_U32: (np.uint32, 1),
    ENTITY_ID_I32: (np.int32, 1),
    DISTANCE_F32: (np.float32, 1),
    AZIMUTH_F32: (np.float32, 1),
    ELEVATION_F32: (np.float32, 1),
    RING_ID_U16: (np.uint16, 1),
    TIME_STAMP_F64: (np.float64, 1),
    ABSOLUTE_VELOCITY_VEC3_F32: (np.float32, 3),
    RELATIVE_VELOCITY_VEC3_F32: (np.float32, 3),
    RADIAL_SPEED_F32: (np.float32, 1),
    NORMAL_VEC3_F32: (np.float32, 3),
    INCIDENT_ANGLE_F32: (np.float32, 1),
    LASER_RETRO_F32: (np.float32, 1),
}

LOG_WARN = 3

_lib = C.CDLL(os.path.abspath(_LIB_PATH))
_vp = C.c_void_p
_pvp = C.POINTER(C.c_void_p)


def _sig(name, *args):
    f = getattr(_lib, name)
    f.argtypes = list(args)
    f.restype = C.c_int32
    return f


_get_version = _sig('rgl_get_version_info', C.POINTER(C.c_int32), C.POINTER(C.c_int32), C.POINTER(C.c_int32))
_configure_logging = _sig('rgl_configure_logging', C.c_int32, C.c_char_p, C.c_bool)
_lib.rgl_get_last_error_string.argtypes = [C.POINTER(C.c_char_p)]
_lib.rgl_get_last_error_string.restype = None
_mesh_create = _sig('rgl_mesh_create', _pvp, _vp, C.c_int32, _vp, C.c_int32)
_mesh_destroy = _sig('rgl_mesh_destroy', _vp)
_entity_destroy = _sig('rgl_entity_destroy', _vp)
_graph_destroy = _sig('rgl_graph_destroy', _vp)
_entity_create = _sig('rgl_entity_create', _pvp, _vp, _vp)
_entity_set_transform = _sig('rgl_entity_set_transform', _vp, _vp)
_entity_set_id = _sig('rgl_entity_set_id', _vp, C.c_int32)
_entity_set_ignored = _sig('rgl_entity_set_ignored_by_sensor', _vp, C.c_int32)
_entity_set_laser_retro = _sig('rgl_entity_set_laser_retro', _vp, C.c_float)
_scene_set_time = _sig('rgl_scene_set_time', _vp, C.c_uint64)
_rays_from_mat = _sig('rgl_node_rays_from_mat3x4f', _pvp, _vp, C.c_int32)
_rays_set_range = _sig('rgl_node_rays_set_range', _pvp, _vp, C.c_int32)
_rays_transform = _sig('rgl_node_rays_transform', _pvp, _vp)
_points_transform = _sig('rgl_node_points_transform', _pvp, _vp)
_raytrace = _sig('rgl_node_raytrace', _pvp, _vp)
_raytrace_velocity = _sig('rgl_node_raytrace_configure_velocity', _vp, _vp, _vp)
_raytrace_id = _sig('rgl_node_raytrace_configure_id', _vp, C.c_int32)
_compact = _sig('rgl_node_points_compact_by_field', _pvp, C.c_int32)
_yield = _sig('rgl_node_points_yield', _pvp, _vp, C.c_int32)
_graph_run = _sig('rgl_graph_run', _vp)
_add_child = _sig('rgl_graph_node_add_child', _vp, _vp)
_result_size = _sig('rgl_graph_get_result_size', _vp, C.c_int32, C.POINTER(C.c_int32), C.POINTER(C.c_int32))
_result_data = _sig('rgl_graph_get_result_data', _vp, C.c_int32, _vp)


class RGLError(RuntimeError):
    pass


def _check(status):
    if status != 0:
        s = C.c_char_p()
        _lib.rgl_get_last_error_string(C.byref(s))
        raise RGLError(f'RGL error {status}: {s.value.decode() if s.value else "?"}')


def version():
    a, b, c = C.c_int32(), C.c_int32(), C.c_int32()
    _check(_get_version(C.byref(a), C.byref(b), C.byref(c)))
    return a.value, b.value, c.value


def configure_logging(level=LOG_WARN):
    _check(_configure_logging(level, None, True))


def mat3x4(T):
    """4x4 동차행렬 → RGL rgl_mat3x4f 메모리(float32 3x4, row-major)."""
    return np.ascontiguousarray(np.asarray(T, dtype=np.float64)[:3, :4], dtype=np.float32)


def _ptr(a):
    return a.ctypes.data_as(_vp)


class Mesh:
    def __init__(self, vertices, faces):
        v = np.ascontiguousarray(vertices, dtype=np.float32).reshape(-1, 3)
        f = np.ascontiguousarray(faces, dtype=np.int32).reshape(-1, 3)
        self.h = _vp()
        _check(_mesh_create(C.byref(self.h), _ptr(v), len(v), _ptr(f), len(f)))

    def destroy(self):
        if self.h:
            _check(_mesh_destroy(self.h))
            self.h = _vp()


class Entity:
    def __init__(self, mesh, entity_id=None, laser_retro=None, ignored_by_sensor=None):
        self.h = _vp()
        _check(_entity_create(C.byref(self.h), None, mesh.h))
        if entity_id is not None:
            _check(_entity_set_id(self.h, int(entity_id)))
        if laser_retro is not None:
            _check(_entity_set_laser_retro(self.h, float(laser_retro)))
        if ignored_by_sensor is not None:
            _check(_entity_set_ignored(self.h, int(ignored_by_sensor)))

    def set_transform(self, T):
        m = mat3x4(T)
        _check(_entity_set_transform(self.h, _ptr(m)))

    def destroy(self):
        if self.h:
            _check(_entity_destroy(self.h))
            self.h = _vp()


def set_scene_time(ns):
    _check(_scene_set_time(None, C.c_uint64(int(ns))))


class RaycastGraph:
    """rays(센서 좌표) → 범위 → 센서 포즈 변환 → raytrace → hit만 남김 → 센서 좌표로 역변환 → yield.

    결과 XYZ는 센서 좌표계로 돌려준다.
    """

    def __init__(self, ray_poses, ranges, fields, sensor_id):
        self.fields = list(fields)
        self.n_rays = 0
        self.rays = _vp()
        self.range = _vp()
        self.tf_rays = _vp()
        self.trace = _vp()
        self.compact = _vp()
        self.tf_points = _vp()
        self.yld = _vp()
        self.set_rays(ray_poses, ranges)
        eye = np.eye(4)
        _check(_rays_transform(C.byref(self.tf_rays), _ptr(mat3x4(eye))))
        _check(_raytrace(C.byref(self.trace), None))
        _check(_raytrace_id(self.trace, int(sensor_id)))
        _check(_compact(C.byref(self.compact), IS_HIT_I32))
        _check(_points_transform(C.byref(self.tf_points), _ptr(mat3x4(eye))))
        fa = np.array(self.fields, dtype=np.int32)
        _check(_yield(C.byref(self.yld), _ptr(fa), len(fa)))
        for a, b in [(self.rays, self.range), (self.range, self.tf_rays), (self.tf_rays, self.trace),
                     (self.trace, self.compact), (self.compact, self.tf_points), (self.tf_points, self.yld)]:
            _check(_add_child(a, b))

    def destroy(self):
        if self.rays:
            _check(_graph_destroy(self.rays))
            self.rays = _vp()

    def set_rays(self, ray_poses, ranges):
        """ray_poses: (N,4,4) 센서 좌표계 ray 포즈(+Z가 ray 방향). ranges: (N,2) 또는 (2,)."""
        m = np.ascontiguousarray(np.asarray(ray_poses)[:, :3, :4], dtype=np.float32)
        self.n_rays = len(m)
        _check(_rays_from_mat(C.byref(self.rays), _ptr(m), len(m)))
        r = np.ascontiguousarray(np.asarray(ranges, dtype=np.float32).reshape(-1, 2))
        _check(_rays_set_range(C.byref(self.range), _ptr(r), len(r)))

    def run(self, T_world_sensor, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)):
        T = np.asarray(T_world_sensor, dtype=np.float64)
        _check(_rays_transform(C.byref(self.tf_rays), _ptr(mat3x4(T))))
        _check(_points_transform(C.byref(self.tf_points), _ptr(mat3x4(np.linalg.inv(T)))))
        lv = np.asarray(lin_vel, dtype=np.float32)
        av = np.asarray(ang_vel, dtype=np.float32)
        _check(_raytrace_velocity(self.trace, _ptr(lv), _ptr(av)))
        _check(_graph_run(self.rays))
        out = {}
        for f in self.fields:
            cnt, size = C.c_int32(), C.c_int32()
            _check(_result_size(self.yld, f, C.byref(cnt), C.byref(size)))
            dt, k = FIELD_DTYPE[f]
            buf = np.empty((cnt.value, k) if k > 1 else (cnt.value,), dtype=dt)
            if cnt.value:
                _check(_result_data(self.yld, f, _ptr(buf)))
            out[f] = buf
        return out


def ray_poses_from_angles(azimuth, elevation, origin=(0, 0, 0)):
    """방위각(반시계, x축 기준)/고각(위가 +) → +Z가 ray 방향인 4x4 포즈 배열."""
    az = np.asarray(azimuth, dtype=np.float64)
    el = np.asarray(elevation, dtype=np.float64)
    d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], -1)
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(up, d)
    bad = np.linalg.norm(x, axis=1) < 1e-9
    x[bad] = [1.0, 0.0, 0.0]
    x /= np.linalg.norm(x, axis=1, keepdims=True)
    y = np.cross(d, x)
    T = np.zeros((len(az), 4, 4))
    T[:, :3, 0] = x
    T[:, :3, 1] = y
    T[:, :3, 2] = d
    T[:, :3, 3] = origin
    T[:, 3, 3] = 1
    return T
