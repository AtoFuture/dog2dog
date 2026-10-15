from setuptools import setup

setup(
    name="baiyanggo_webrtc_bridge",
    version="0.1.0",
    description="Go2 内置雷达 WebRTC→ROS2 桥",
    maintainer="Shuuyou",
    maintainer_email="shuuyou@outlook.com",
    license="Apache-2.0",
    packages=["baiyanggo_webrtc_bridge"],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "webrtc_bridge = baiyanggo_webrtc_bridge.bridge_node:main",
        ],
    },
)
