from setuptools import find_packages, setup


package_name = "g3_integration_eval"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="G3 Team",
    maintainer_email="g3-team@example.com",
    description=(
        "G3 integration state machine, low-battery RETURN monitor, and a "
        "safety-gated NavigateToPose client with explicit outcome events."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "g3_state_machine = g3_integration_eval.state_machine_node:main",
            "evaluate_bag = g3_integration_eval.evaluate_bag:main",
            "mission_brain = g3_integration_eval.mission_brain_node:main",
        ],
    },
)
