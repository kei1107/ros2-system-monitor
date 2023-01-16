#!/usr/bin/env python3

#################################################################################
# Copyright 2009, Willow Garage, Inc.
# Copyright 2013 by Ralf Kaestner
# Copyright 2013 by Jerome Maye
# Copyright 2023 by kei1107
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#################################################################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file_default = os.path.join(get_package_share_directory(
        "ros2_system_monitor"), "config", "system_monitor.yaml")
    declare_config_file = DeclareLaunchArgument(
        name="system_monitor_config_file", default_value=config_file_default,
        description="system monitor config file path")
    declare_enable_cpu_monitor = DeclareLaunchArgument(
        name="enable_cpu_monitor", default_value="true", description="enable cpu_monitor if true")
    declare_enable_hdd_monitor = DeclareLaunchArgument(
        name="enable_hdd_monitor", default_value="true", description="enable hdd_monitor if true")
    declare_enable_mem_monitor = DeclareLaunchArgument(
        name="enable_mem_monitor", default_value="true", description="enable mem_monitor if true")
    declare_enable_net_monitor = DeclareLaunchArgument(
        name="enable_net_monitor", default_value="true", description="enable net_monitor if true")
    declare_enable_ntp_monitor = DeclareLaunchArgument(
        name="enable_ntp_monitor", default_value="true", description="enable ntp_monitor if true")

    cpu_monitor = Node(
        package="ros2_system_monitor",
        executable="cpu_monitor_node.py",
        parameters=[LaunchConfiguration("system_monitor_config_file")],
        respawn=True,
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_cpu_monitor"))
    )
    hdd_monitor = Node(
        package="ros2_system_monitor",
        executable="hdd_monitor_node.py",
        parameters=[LaunchConfiguration("system_monitor_config_file")],
        respawn=True,
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_hdd_monitor"))
    )
    mem_monitor = Node(
        package="ros2_system_monitor",
        executable="mem_monitor_node.py",
        parameters=[LaunchConfiguration("system_monitor_config_file")],
        respawn=True,
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_mem_monitor"))
    )
    net_monitor = Node(
        package="ros2_system_monitor",
        executable="net_monitor_node.py",
        parameters=[LaunchConfiguration("system_monitor_config_file")],
        respawn=True,
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_net_monitor"))
    )
    ntp_monitor = Node(
        package="ros2_system_monitor",
        executable="ntp_monitor_node.py",
        parameters=[LaunchConfiguration("system_monitor_config_file")],
        respawn=True,
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_ntp_monitor"))
    )

    # create the launch description
    ld = LaunchDescription()

    # declare the launch options
    ld.add_action(declare_config_file)
    ld.add_action(declare_enable_cpu_monitor)
    ld.add_action(declare_enable_hdd_monitor)
    ld.add_action(declare_enable_mem_monitor)
    ld.add_action(declare_enable_net_monitor)
    ld.add_action(declare_enable_ntp_monitor)

    # add the actions
    ld.add_action(cpu_monitor)
    ld.add_action(hdd_monitor)
    ld.add_action(mem_monitor)
    ld.add_action(net_monitor)
    ld.add_action(ntp_monitor)

    return ld
