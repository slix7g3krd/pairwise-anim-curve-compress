"""角色跑步动画曲线压缩工具。

修复要点：峰值误差约束、四元数归一化、循环边界速度对齐、多轨道相位
联合评估、根运动高精度量化、事件帧锁定、按帧率采样、包围盒重算、
镜像曲线同步以及原始曲线备份。
"""

import copy
import math

# 循环边界速度比较的默认绝对容差。
LOOP_VELOCITY_TOLERANCE = 1e-6


def error_ok(error, tolerance):
    """判断压缩误差是否在容差内。

    误差取绝对值，因此峰值处的正负偏差都会被计入，避免误差累积到
    下一段。
    """
    if tolerance < 0:
        return False
    return abs(error) <= tolerance


def normalize_quat(quat):
    """归一化四元数。

    逐分量插值后的四元数长度可能不再是 1，直接用于旋转会引入缩放；
    这里按模长缩放回单位四元数。零四元数无法归一化，原样返回。
    """
    if len(quat) != 4:
        raise ValueError("quaternion must have 4 components")
    norm = math.sqrt(sum(component * component for component in quat))
    if norm == 0:
        return tuple(float(component) for component in quat)
    return tuple(float(component) / norm for component in quat)


def loop_ok(start_velocity, end_velocity):
    """判断循环边界速度是否连续。"""
    return math.isclose(
        start_velocity,
        end_velocity,
        rel_tol=0.0,
        abs_tol=LOOP_VELOCITY_TOLERANCE,
    )


def phase_diff(left_phase, right_phase):
    """返回左右轨道相位差（绝对值）。"""
    return abs(left_phase - right_phase)


def root_precision(value, step):
    """按步长量化根运动。

    使用调用方给定的步长而不是固定的整厘米，长距离位移不会因粗量化
    而累积漂移。步长非正时不做量化。
    """
    if step <= 0:
        return value
    return round(value / step) * step


def lock_event(event_time, key_time, threshold):
    """把事件帧锁到关键帧。

    仅当事件帧与关键帧的距离在阈值内才吸附到关键帧，否则保留原事件
    时间，避免脚步声被移动到邻近但无关的关键帧。
    """
    if abs(event_time - key_time) <= threshold:
        return key_time
    return event_time


def sample_rate_scale(source_fps, target_fps):
    """按帧率返回采样倍率。

    不同帧率的动画使用各自的采样表：目标帧率相对源帧率的倍数即为
    采样倍率。
    """
    if source_fps <= 0:
        raise ValueError("source_fps must be positive")
    return target_fps / source_fps


def recompute_bounds(min_value, max_value, margin):
    """重算动画包围盒。

    压缩后曲线的极值可能变化，需要用压缩后的最小/最大值加外边距
    重新计算包围盒，避免角色被错误剔除。
    """
    if margin < 0:
        raise ValueError("margin must be non-negative")
    return (min_value - margin, max_value + margin)


def mirror_curve(values):
    """同步镜像曲线。

    返回与输入等长且顺序反转的新列表，供镜像动画与源曲线同步压缩，
    保证左右两侧对称。
    """
    return list(values)[::-1]


def backup_original(curve):
    """保留原始曲线副本。

    深拷贝输入曲线，压缩失败时可以从该副本回退。
    """
    return copy.deepcopy(curve)
