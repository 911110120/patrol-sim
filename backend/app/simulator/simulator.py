"""仿真核心：持有内存地图与机器人状态，处理移动、规划、感知与自动巡检。

本模块不依赖 HTTP 或浏览器，规则可脱离接口独立测试。共享状态由一把锁保护，
后台巡检线程与接口请求通过同一把锁串行访问内部数据。
"""

import threading
import time

from . import planner, sensor
from .world import DIRECTIONS, DIRECTION_LABELS, Position, World, default_world


class SimError(Exception):
    """仿真规则错误，携带供接口层映射的错误码和中文提示。"""

    def __init__(self, code: str, message: str, http_status: int) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.http_status = http_status


def _direction_between(a: Position, b: Position) -> str:
    dx, dy = b.x - a.x, b.y - a.y
    for name, vec in DIRECTIONS.items():
        if vec == (dx, dy):
            return name
    return "right"


class Simulator:
    """同一服务实例共享一个机器人状态，不按浏览器划分会话。"""

    def __init__(self, world: World | None = None, tick_seconds: float = 0.3) -> None:
        self._world = world or default_world()
        self._lock = threading.Lock()
        self._robot_pos = self._world.start
        self._robot_dir = "right"
        self._robot_status = "idle"
        self._visited: list[Position] = []
        self._patrol_active = False
        self._patrol_completed = False
        self._patrol_path: list[Position] = []
        self._patrol_target: Position | None = None
        self._patrol_message = ""

        self._stop_event = threading.Event()
        self._tick = tick_seconds
        self._thread = threading.Thread(target=self._patrol_loop, daemon=True)
        self._thread.start()

    # ---------- 状态读取 ----------

    def robot_position(self) -> Position:
        with self._lock:
            return self._robot_pos

    def state(self) -> dict:
        with self._lock:
            return self._snapshot()

    def _snapshot(self) -> dict:
        return {
            "map": self._world.to_dict(),
            "robot": {
                "x": self._robot_pos.x,
                "y": self._robot_pos.y,
                "dir": self._robot_dir,
                "dirLabel": DIRECTION_LABELS[self._robot_dir],
                "status": self._robot_status,
            },
            "sensor": sensor.scan(self._world, self._robot_pos, self._robot_dir),
            "patrol": {
                "active": self._patrol_active,
                "completed": self._patrol_completed,
                "visited": [p.as_dict() for p in self._visited],
                "target": self._patrol_target.as_dict() if self._patrol_target else None,
                "message": self._patrol_message,
            },
        }

    # ---------- 手动控制 ----------

    def move(self, direction: str) -> dict:
        with self._lock:
            if direction not in DIRECTIONS:
                self._robot_status = "blocked"
                raise SimError("invalid_request", "方向有误，请使用上、下、左、右四个方向", 400)

            dx, dy = DIRECTIONS[direction]
            target = Position(self._robot_pos.x + dx, self._robot_pos.y + dy)

            if not self._world.in_bounds(target):
                self._robot_status = "blocked"
                self._stop_patrol_locked()
                raise SimError("out_of_bounds", "已到地图边界，无法继续向该方向移动", 409)
            if self._world.is_obstacle(target):
                self._robot_status = "blocked"
                self._stop_patrol_locked()
                raise SimError("obstacle", "前方有障碍物，无法移动", 409)

            # 校验全部通过后才更新位置；被拒绝的指令不会改变状态。
            self._robot_pos = target
            self._robot_dir = direction
            self._robot_status = "idle"
            self._record_checkpoint(target)
            # 手动操作视为人工接管，终止正在进行的自动巡检。
            self._stop_patrol_locked()
            return self._snapshot()

    # ---------- 路径规划 ----------

    def plan(self, start: Position, goal: Position) -> dict:
        with self._lock:
            if not self._world.in_bounds(start) or not self._world.in_bounds(goal):
                return {"ok": False, "path": [], "reason": "起点或目标点在地图范围之外"}
            if self._world.is_obstacle(start) or self._world.is_obstacle(goal):
                return {"ok": False, "path": [], "reason": "起点或目标点位于障碍物上"}
            path = planner.plan(self._world, start, goal)
            if path is None:
                return {"ok": False, "path": [], "reason": "目标点不可达，中间被边界或障碍物隔开"}
            return {"ok": True, "path": [p.as_dict() for p in path], "reason": ""}

    # ---------- 自动巡检 ----------

    def patrol_start(self) -> dict:
        with self._lock:
            if self._patrol_active:
                self._patrol_message = "巡检已在进行中"
                return self._snapshot()
            self._visited = []
            self._patrol_completed = False
            self._patrol_path = []
            self._patrol_target = None
            self._patrol_active = True
            self._patrol_message = "正在按顺序前往检查点…"
            self._robot_status = "moving"
            return self._snapshot()

    def patrol_stop(self) -> dict:
        with self._lock:
            self._stop_patrol_locked()
            self._robot_status = "idle"
            self._patrol_message = "巡检已手动停止"
            return self._snapshot()

    def reset(self) -> dict:
        with self._lock:
            self._robot_pos = self._world.start
            self._robot_dir = "right"
            self._robot_status = "idle"
            self._visited = []
            self._stop_patrol_locked()
            self._patrol_completed = False
            self._patrol_message = "已复位到初始位置"
            return self._snapshot()

    # ---------- 巡检后台线程 ----------

    def _patrol_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                self._patrol_tick_locked()
            time.sleep(self._tick)

    def _patrol_tick_locked(self) -> None:
        if not self._patrol_active:
            return

        # 1. 沿已规划路径前进一步。
        if self._patrol_path:
            nxt = self._patrol_path.pop(0)
            if not self._world.passable(nxt):
                self._patrol_active = False
                self._robot_status = "error"
                self._patrol_message = "路径受阻，巡检已停止"
                return
            self._robot_dir = _direction_between(self._robot_pos, nxt)
            self._robot_pos = nxt
            self._robot_status = "moving"
            self._record_checkpoint(nxt)
            self._refresh_patrol_locked()
            return

        # 2. 路径走完，规划到下一个未访问的检查点。
        target = next((c for c in self._world.checkpoints if c not in self._visited), None)
        if target is None:
            self._patrol_active = False
            self._patrol_completed = True
            self._robot_status = "idle"
            self._patrol_message = "巡检完成：全部检查点均已到达"
            return

        path = planner.plan(self._world, self._robot_pos, target)
        if path is None:
            self._patrol_active = False
            self._robot_status = "error"
            self._patrol_target = target
            self._patrol_message = f"检查点 ({target.x}, {target.y}) 不可达，巡检已停止"
            return

        self._patrol_target = target
        self._patrol_path = path[1:]  # 去掉起点本身

    def _record_checkpoint(self, pos: Position) -> None:
        if pos in self._world.checkpoints and pos not in self._visited:
            self._visited.append(pos)

    def _refresh_patrol_locked(self) -> None:
        if len(self._visited) == len(self._world.checkpoints):
            self._patrol_active = False
            self._patrol_completed = True
            self._robot_status = "idle"
            self._patrol_message = "巡检完成：全部检查点均已到达"

    def _stop_patrol_locked(self) -> None:
        self._patrol_active = False
        self._patrol_path = []
        self._patrol_target = None

    def close(self) -> None:
        """服务关闭时结束后台线程。"""
        self._stop_event.set()
        self._thread.join(timeout=1)
