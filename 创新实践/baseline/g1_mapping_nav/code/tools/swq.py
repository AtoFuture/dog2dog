import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
import threading


class S(Node):
    def __init__(self):
        super().__init__('sw_q')
        self.cli = self.create_client(
            SwitchController, '/robot1/controller_manager/switch_controller')

    def run(self):
        self.cli.wait_for_service(timeout_sec=20.0)
        r = SwitchController.Request()
        r.activate_controllers = ['joint_state_broadcaster', 'joint_group_controller']
        r.deactivate_controllers = []
        r.strictness = 1
        r.timeout.sec = 30
        f = self.cli.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=40.0)
        rr = f.result()
        with open('/tmp/switch_res.txt', 'w') as fh:
            fh.write("OK=%s\n" % (rr.ok if rr else None))


def main():
    rclpy.init()
    n = S()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    n.run()


if __name__ == '__main__':
    main()
