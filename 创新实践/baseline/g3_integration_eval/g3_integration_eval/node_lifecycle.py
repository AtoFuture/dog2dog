"""Defer ROS context shutdown until the current DDS callback has returned."""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def run_node(factory, args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopping = False

    def request_stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    # Keep these installed through destruction: another stop signal must not
    # interrupt teardown or asynchronously invalidate a DDS take operation.
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = None
    try:
        node = factory()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown(uninstall_handlers=False)
