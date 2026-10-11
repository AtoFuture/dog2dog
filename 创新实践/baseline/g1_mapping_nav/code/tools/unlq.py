import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import UnloadController, SwitchController
import threading

res = {}


class S(Node):
    def __init__(self):
        super().__init__('unload_q')
        self.un = self.create_client(UnloadController, '/robot1/controller_manager/unload_controller')

    def run(self):
        self.un.wait_for_service(timeout_sec=20.0)
        r = UnloadController.Request()
        r.name = 'joint_state_broadcaster'
        f = self.un.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=30.0)
        rr = f.result()
        with open('/tmp/unload_res.txt', 'w') as fh:
            fh.write("LOADED=%s\n" % (getattr(rr, 'ok', None) if rr else None))


def main():
    rclpy.init()
    n = S()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    n.run()


if __name__ == '__main__':
    main()
