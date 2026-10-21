import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


class Relay(Node):
    def __init__(self):
        super().__init__('map_relay')
        qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_map = self.create_publisher(OccupancyGrid, '/robot1/map', qos)
        self.pub_upd = self.create_publisher(OccupancyGrid, '/robot1/map_updates', qos)
        self.n_map = 0
        self.n_upd = 0
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, qos)
        self.create_subscription(OccupancyGrid, '/map_updates', self.cb_upd, qos)
        self.timer = self.create_timer(30.0, self.log)

    def cb_map(self, m):
        self.n_map += 1
        self.pub_map.publish(m)

    def cb_upd(self, m):
        self.n_upd += 1
        self.pub_upd.publish(m)

    def log(self):
        self.get_logger().info('relay map=%d upd=%d' % (self.n_map, self.n_upd))


def main():
    rclpy.init()
    n = Relay()
    rclpy.spin(n)


if __name__ == '__main__':
    main()
