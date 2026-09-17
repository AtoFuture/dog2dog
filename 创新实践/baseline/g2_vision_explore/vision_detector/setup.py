from setuptools import setup

package_name = "vision_detector"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/detector.yaml"]),
        ("share/" + package_name + "/launch", ["launch/detector.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AtoFuture",
    maintainer_email="240923267+AtoFuture@users.noreply.github.com",
    description="G2 感知链路：检测 + 三维解算 + 发布 Detection3D",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "detector_node = vision_detector.detector_node:main",
            "replay_images = vision_detector.replay_images:main",
        ],
    },
)
