#!/usr/bin/env python3
"""在仓库外构建 MAVROS 2.15.1 官方 odometry 插件；不替换系统 extras。"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.request
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET

SOURCES={
    "odom.cpp":("mavros_extras/src/plugins/odom.cpp","181ff4710a5f6e9c28d6a1a84e197584dbc825a7f6914f85abad81c3c4217133"),
    "LICENSE-BSD.txt":("LICENSE-BSD.txt","a2b0498e8df6e1f25d2d6b6f0d52363cc6fca8dc308dc9b4a6d19538f5817d9c"),
    "LICENSE.md":("LICENSE.md","89a9a4fc3cf00cf4364ee1ab726325f8d2573327fa44ecc9ed1590232e939b2c"),
}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args();base=args.out.expanduser().resolve()
    workspace=Path(__file__).resolve().parents[1]
    repository=workspace.parents[1] if (workspace.parents[1]/".git").exists() else workspace
    if base.is_relative_to(repository):
        parser.error("build output must be outside the repository")
    installed=ET.parse(Path(get_package_share_directory("mavros"))/"package.xml").getroot().findtext("version")
    if installed!="2.15.1":raise RuntimeError("requires installed MAVROS 2.15.1, got "+str(installed))
    package=base/"source/bb_mavros_odometry";package.mkdir(parents=True,exist_ok=True)
    provenance={"mavros_version":installed,"plugin_name":"bb_odometry","sources":{}}
    for name,(path,expected) in SOURCES.items():
        target=package/name;url="https://raw.githubusercontent.com/mavlink/mavros/2.15.1/"+path
        data=target.read_bytes() if target.exists() else urllib.request.urlopen(url,timeout=30).read()
        digest=hashlib.sha256(data).hexdigest()
        if digest!=expected:raise RuntimeError("source checksum mismatch: "+name)
        target.write_bytes(data);provenance["sources"][name]={"url":url,"sha256":digest}
    (package/"package.xml").write_text('''<package format="3">
  <name>bb_mavros_odometry</name><version>0.1.0</version>
  <description>MAVROS 2.15.1 odometry plugin, isolated build.</description>
  <maintainer email="waterc07@users.noreply.github.com">waterc07</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>mavros</depend><depend>nav_msgs</depend><depend>tf2_eigen</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
''')
    (package/"plugins.xml").write_text('''<library path="bb_mavros_odometry">
  <class name="bb_odometry" type="mavros::plugin::PluginFactoryTemplate&lt;mavros::extra_plugins::OdometryPlugin&gt;" base_class_type="mavros::plugin::PluginFactory">
    <description>MAVROS 2.15.1 odometry.</description>
  </class>
</library>
''')
    (package/"CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.16)
project(bb_mavros_odometry)
set(CMAKE_CXX_STANDARD 20)
find_package(ament_cmake REQUIRED)
find_package(mavros REQUIRED)
find_package(mavlink REQUIRED)
find_package(libmavconn REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(eigen_stl_containers REQUIRED)

add_library(bb_mavros_odometry SHARED odom.cpp)
ament_target_dependencies(bb_mavros_odometry mavros mavlink libmavconn nav_msgs tf2_eigen tf2_ros pluginlib rcpputils rclcpp diagnostic_msgs diagnostic_updater mavros_msgs sensor_msgs std_msgs geometry_msgs geographic_msgs eigen_stl_containers)
target_include_directories(bb_mavros_odometry PRIVATE ${EIGEN3_INCLUDE_DIRS})
pluginlib_export_plugin_description_file(mavros plugins.xml)
install(TARGETS bb_mavros_odometry LIBRARY DESTINATION lib)
install(FILES LICENSE.md LICENSE-BSD.txt DESTINATION share/${PROJECT_NAME})
ament_package()
''')
    (base/"provenance.json").write_text(json.dumps(provenance,indent=2)+"\n")
    subprocess.run(["colcon","--log-base",str(base/"log"),"build","--base-paths",str(base/"source"),
        "--build-base",str(base/"build"),"--install-base",str(base/"install"),
        "--packages-select","bb_mavros_odometry","--cmake-args","-DBUILD_TESTING=OFF","-DCMAKE_BUILD_TYPE=Release"],check=True)
    print("source "+str(base/"install/local_setup.bash"))


if __name__=="__main__":main()
