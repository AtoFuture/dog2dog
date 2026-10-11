import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
import threading

res = {}


class S(Node):
    def __init__(self):
        super().__init__('lcq')
        self.cli = self.create_client(ListControllers, '/robot1/controller_manager/list_controllers')

    def run(self):
        self.cli.wait_for_service(timeout_sec=20.0)
        f = self.cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, f, timeout_sec=30.0)
        rr = f.result()
        if rr:
            with open('/tmp/ctrl_states.txt', 'w') as fh:
                for c in rr.controller:
                    fh.write("%s %s\n" % (c.name, c.state))
        else:
            with open('/tmp/ctrl_states.txt', 'w') as fh:
                fh.write("NO_RESULT\n")


def main():
    rclpy.init()
    n = S()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    n.run()


if __name__ == '__main__':
    main()
