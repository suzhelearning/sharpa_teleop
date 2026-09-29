from glob import glob
from setuptools import find_packages, setup

setup(
    name="sharpa_teleop", version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sharpa_teleop"]),
        ("share/sharpa_teleop", ["package.xml"]),
        ("share/sharpa_teleop/launch", glob("launch/*.launch.py")),
        ("share/sharpa_teleop/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"], zip_safe=False,
    maintainer="Fcl", maintainer_email="58226448@qq.com",
    description="Manus to SharpaWave ROS 2 teleoperation", license="Proprietary",
    entry_points={"console_scripts": [
        "manus_input = sharpa_teleop.manus_input:main",
        "retarget = sharpa_teleop.retarget:main",
        "sharpa_output = sharpa_teleop.sharpa_output:main",
        "mujoco_sim = sharpa_teleop.mujoco_sim:main",
    ]},
)
