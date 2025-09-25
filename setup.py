from setuptools import find_packages, setup

package_name = "ros_behaviors_fsm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bhargavi",
    maintainer_email="bhargavi.a.deshpande@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "draw_triangle = ros_behaviors_fsm.draw_triangle:main",
            "wall_following = ros_behaviors_fsm.wall_follower:main",
            "teleop = ros_behaviors_fsm.teleop:main"
        ],
    },
)
