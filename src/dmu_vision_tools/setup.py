from setuptools import setup

package_name = "dmu_vision_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/expected_topics.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="team3",
    maintainer_email="design3-team3@proton.me",
    description="Diagnostics for the vision subsystem",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_health_check = dmu_vision_tools.health_check:main",
            "measure_detection = dmu_vision_tools.measure_detection:main",
        ],
    },
)
