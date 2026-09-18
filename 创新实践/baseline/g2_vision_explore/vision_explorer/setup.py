from setuptools import setup

package_name = "vision_explorer"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AtoFuture",
    maintainer_email="240923267+AtoFuture@users.noreply.github.com",
    description="G2 探索决策的 ROS2 节点与测试工具",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 交付物 2 的接线：frontier 选点 → 发 NavigateToPose
            "explorer_node = vision_explorer.explorer_node:main",
            # 假的 NavigateToPose 服务器 —— 无 G1 / 无 Nav2 / 无真机时调状态机用
            "fake_goal_server = vision_explorer.fake_goal_server:main",
            # 抢占探针 —— 实测「被抢占的 goal 客户端看到什么状态码」
            "preemption_probe = vision_explorer.preemption_probe:main",
        ],
    },
)
