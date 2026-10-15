"""逻辑相机真值位姿缓存(纯逻辑,与 ROS 解耦,便于单元测试)。

俯视安装(rpy = 0, 90°, 0)的逻辑相机,传感器系读数与世界系的换算
(与 attach_helper.GzKinematicAttach._on_logical 同款):

    world_x = cam_x + rel_z
    world_y = cam_y + rel_y

(rel_x 是深度分量,有系统偏差不可信,弃用;详见 attach_helper 模块注释。)

执行节点 pose_from_camera=true 时,任务开始用缓存里的 (true_x, true_y)
配合 feed_z 常量作为取件中心,替代 PartInfo.pose 的 xy;缓存无该件时
抛 GraspError('no_camera_pose')(可重试语义:稍后相机刷新即可命中)。
"""


def world_xy_from_rel(camera_xy: tuple[float, float],
                      rel_x: float, rel_y: float, rel_z: float
                      ) -> tuple[float, float]:
    """传感器系相对位姿 -> 世界系俯视 xy(换算式见模块注释,rel_x 弃用)。"""
    return (camera_xy[0] + rel_z, camera_xy[1] + rel_y)


class CameraPoseCache:
    """part_id -> 世界系真值 xy 的滚动缓存(只缓存 part_ 开头的模型名)。"""

    def __init__(self, camera_xy: tuple[float, float] = (0.65, 0.0)):
        self._cam_xy = (float(camera_xy[0]), float(camera_xy[1]))
        self._poses: dict[str, tuple[float, float]] = {}

    @property
    def camera_xy(self) -> tuple[float, float]:
        return self._cam_xy

    def update(self, models) -> None:
        """喂入一批 (name, rel_x, rel_y, rel_z);非 part_ 前缀的模型忽略。"""
        for name, rel_x, rel_y, rel_z in models:
            name = str(name)
            if not name.startswith('part_'):
                continue
            self._poses[name] = world_xy_from_rel(
                self._cam_xy, rel_x, rel_y, rel_z)

    def get(self, part_id: str) -> tuple[float, float] | None:
        """返回该件最新缓存真值 xy;无缓存返回 None(调用方走可重试失败)。"""
        return self._poses.get(part_id)

    def known_parts(self) -> set[str]:
        return set(self._poses)
