from setuptools import setup

# ⚠️ 这个包有点特殊：Python 源码**直接放在本目录下**（g2_core/*.py），
# 而不是 ament_python 惯例的嵌套目录（g2_core/g2_core/*.py）。
#
# 这样做的原因：g2_core 是「不需要 ROS2 也能开发和测试」的算法内核，
# 它的 pytest 直接以本目录为 pythonpath。嵌套一层会让 pytest 配置变成
# 需要同时加两个路径，反而 confusing。
#
# package_dir 把「包名 g2_core」映射到当前目录即可，colcon 能正确处理。
setup(
    name="g2_core",
    version="0.1.0",
    packages=["g2_core", "g2_core.config"],
    package_dir={"g2_core": "."},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/g2_core"]),
        ("share/g2_core", ["package.xml"]),
    ],
    # 跟踪器配置要一起装 —— ultralytics 是按**路径**读它的，
    # 不装的话 Detector 会在运行时找不到文件，而错误信息来自 ultralytics 内部，
    # 不会提示「你少装了一个数据文件」。
    package_data={"g2_core": ["config/*.yaml"]},
    include_package_data=True,
    zip_safe=False,
    maintainer="AtoFuture",
    maintainer_email="240923267+AtoFuture@users.noreply.github.com",
    description="G2 算法内核（纯 Python，不依赖 ROS2）",
    license="Apache-2.0",
)
