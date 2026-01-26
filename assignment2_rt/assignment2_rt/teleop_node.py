import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from assignment2_rt_interfaces.srv import GetVelAvg

Y = "\033[33m"
RST = "\033[0m"

def _read_float(prompt: str) -> float:
    while True:
        s = input(f"{Y}{prompt}{RST}").strip()
        try:
            v = float(s)
            if math.isnan(v) or math.isinf(v):
                print(f"{Y}Valore non valido (nan/inf). Riprova.{RST}")
                continue
            return v
        except ValueError:
            print(f"{Y}Input non valido. Inserisci un numero (es: 0.2).{RST}")

class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")

        self.pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)

        # Manteniamo una finestra di campioni per la media (semplice e sufficiente)
        self._lin_hist = deque(maxlen=200)
        self._ang_hist = deque(maxlen=200)

        self.srv = self.create_service(GetVelAvg, "/get_vel_avg", self.on_get_vel_avg)

        self.get_logger().info("teleop_node ready.")
        self.get_logger().info("Inserirai prima linear, poi angular. Ctrl+C per uscire.")

    def on_get_vel_avg(self, req, res):
        if len(self._lin_hist) == 0:
            res.avg_linear = 0.0
            res.avg_angular = 0.0
            return res

        res.avg_linear = float(sum(self._lin_hist) / len(self._lin_hist))
        res.avg_angular = float(sum(self._ang_hist) / len(self._ang_hist))
        return res

    def publish_cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

        self._lin_hist.append(float(linear))
        self._ang_hist.append(float(angular))

        self.get_logger().info(f"Published /cmd_vel: linear={linear:.3f}, angular={angular:.3f}")

def main():
    rclpy.init()
    node = TeleopNode()

    try:
        while rclpy.ok():
            lin = _read_float("Linear velocity (m/s): ")
            ang = _read_float("Angular velocity (rad/s): ")
            node.publish_cmd(lin, ang)
    except KeyboardInterrupt:
        pass
    finally:
        # shutdown “pulito” (evita RCLError da doppio shutdown)
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == "__main__":
    main()
