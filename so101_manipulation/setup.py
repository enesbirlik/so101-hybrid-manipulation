from glob import glob

from setuptools import find_packages, setup

package_name = "so101_manipulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/test", glob("test/smoke_mock.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Enes Birlik",
    maintainer_email="76901870+enesbirlik@users.noreply.github.com",
    description="Cube pick and place for the SO-101: PickCube / PlaceCube action server with a soft grasp.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pick_place = so101_manipulation.pick_place_node:main",
            "so101_cube = so101_manipulation.cli:main",
            "fake_cube_publisher = so101_manipulation.fake_cube_publisher:main",
        ],
    },
)
