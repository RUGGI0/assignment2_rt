import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from assignment2_rt_interfaces.msg import ObstacleInfo
from assignment2_rt_interfaces.srv import SetThreshold

def _finite_ranges(scan: LaserScan):
    for r in scan.ranges:
        if r is None:
            continue
        if math.isfinite(r) and r > 0.0:
            yield float(r)

class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_node")

        self.threshold = 0.8  # default
        self.last_scan: Optional[LaserScan] = None
        self.last_teleop_cmd = Twist()

        # I/O
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel_teleop", self.on_cmd_teleop, 10)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_obs = self.create_publisher(ObstacleInfo, "/obstacle_info", 10)

        # Service
        self.srv_thr = self.create_service(SetThreshold, "/set_threshold", self.on_set_threshold)

        # Timers
        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz
        self.timer_obs = self.create_timer(0.10, self.publish_obstacle_info)  # 10 Hz

        self.get_logger().info("safety_node ready. Sub: /scan, /cmd_vel_teleop | Pub: /cmd_vel, /obstacle_info | Srv: /set_threshold")

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def on_cmd_teleop(self, msg: Twist):
        self.last_teleop_cmd = msg

    def on_set_threshold(self, req, res):
        # clamp
        t = float(req.threshold)
        if (not math.isfinite(t)) or t <= 0.0:
            res.success = False
            return res
        self.threshold = max(0.05, min(t, 10.0))
        res.success = True
        self.get_logger().info(f"threshold set to {self.threshold:.3f}")
        return res

    def _min_in_sector(self, scan: LaserScan, a0: float, a1: float) -> float:
        # returns min range within [a0, a1] (radians), inf if none
        amin = scan.angle_min
        inc = scan.angle_increment
        if inc == 0.0:
            return float("inf")

        n = len(scan.ranges)
        def idx(angle: float) -> int:
            i = int(round((angle - amin) / inc))
            return max(0, min(n - 1, i))

        i0 = idx(a0)
        i1 = idx(a1)
        if i0 > i1:
            i0, i1 = i1, i0

        m = float("inf")
        for r in scan.ranges[i0:i1+1]:
            if r is None:
                continue
            if math.isfinite(r) and r > 0.0:
                if r < m:
                    m = float(r)
        return m

    def _compute_obstacle(self) -> Tuple[float, str]:
        # distance = min valid overall; direction = sector of closest obstacle
        if self.last_scan is None:
            return float("inf"), "unknown"

        scan = self.last_scan
        all_min = min(_finite_ranges(scan), default=float("inf"))

        # sectors (rad): right [-90,-30], front [-30,30], left [30,90]
        deg = math.pi / 180.0
        right = self._min_in_sector(scan, -90*deg, -30*deg)
        front = self._min_in_sector(scan, -30*deg,  30*deg)
        left  = self._min_in_sector(scan,  30*deg,  90*deg)

        # choose direction by smallest sector distance
        best = min((front, "front"), (left, "left"), (right, "right"), key=lambda x: x[0])
        direction = best[1] if math.isfinite(best[0]) else "unknown"

        return all_min, direction

    def publish_obstacle_info(self):
        dist, direction = self._compute_obstacle()
        msg = ObstacleInfo()
        msg.distance = float(dist if math.isfinite(dist) else 999.0)
        msg.direction = direction
        msg.threshold = float(self.threshold)
        self.pub_obs.publish(msg)

    def loop(self):
        dist, _ = self._compute_obstacle()

        out = Twist()
        if math.isfinite(dist) and dist < self.threshold:
            # STOP
            out.linear.x = 0.0
            out.angular.z = 0.0
        else:
            # PASS-THROUGH
            out = self.last_teleop_cmd

        self.pub_cmd.publish(out)

def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
