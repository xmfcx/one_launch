# Copyright 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import yaml


def get_vehicle_info(context):
    # TODO(TIER IV): Use Parameter Substitution after we drop Galactic support
    # https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/substitutions/parameter.py
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    nodes = []

    # turn packets into pointcloud as in
    # https://github.com/ros-drivers/velodyne/blob/ros2/velodyne_pointcloud/launch/velodyne_convert_node-VLP16-composed-launch.py
    nodes.append(
        ComposableNode(
            package="awf_velodyne_pointcloud",
            plugin="velodyne_pointcloud::Convert",
            name="velodyne_convert_node",
            parameters=[
                {
                    **create_parameter_dict(
                        "calibration",
                        "min_range",
                        "max_range",
                        "num_points_thresholds",
                        "invalid_intensity",
                        "frame_id",
                        "scan_phase",
                        "view_direction",
                        "view_width",
                        "invalid_point_remove",
                        "invalid_rings",
                        "invalid_angles_start",
                        "invalid_angles_end"
                    ),
                }
            ],
            remappings=[
                ("velodyne_points", "pointcloud_raw"),
                ("velodyne_points_ex", "pointcloud_raw_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/gnss/sbg/ros/twist_with_covariance_stamped"),
                ("~/input/imu", "/sensing/gnss/sbg/ros/imu/data"),
                ("~/input/pointcloud", "pointcloud_raw_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "outlier_filtered/pointcloud"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    component_loader = LoadComposableNodes(
        composable_node_descriptions=nodes,
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    driver_component = ComposableNode(
        package="awf_velodyne_driver",
        plugin="velodyne_driver::VelodyneDriver",
        # node is created in a global context, need to avoid name clash
        name="velodyne_driver",
        parameters=[
            {
                **create_parameter_dict(
                    "device_ip",
                    "gps_time",
                    "read_once",
                    "read_fast",
                    "repeat_delay",
                    "frame_id",
                    "model",
                    "rpm",
                    "port",
                    "pcap",
                    "scan_phase",
                ),
            }
        ],
    )

    target_container = (
        container
        if UnlessCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )

    driver_component_loader = LoadComposableNodes(
        composable_node_descriptions=[driver_component],
        target_container=target_container,
        condition=IfCondition(LaunchConfiguration("launch_driver")),
    )

    return [container, component_loader, driver_component_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("model", description="velodyne model name")
    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg("calibration", description="path to calibration file")
    add_launch_arg("device_ip", "192.168.1.201", "device ip address")
    add_launch_arg("scan_phase", "0.0")
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("container_name", "velodyne_composable_node_container", "container name")
    add_launch_arg("min_range", description="minimum view range")
    add_launch_arg("max_range", description="maximum view range")
    add_launch_arg("pcap", "")
    add_launch_arg("port", "2368", description="device port number")
    add_launch_arg("read_fast", "False")
    add_launch_arg("read_once", "False")
    add_launch_arg("repeat_delay", "0.0")
    add_launch_arg("rpm", "600.0", "rotational frequency")
    add_launch_arg("laserscan_ring", "-1")
    add_launch_arg("laserscan_resolution", "0.007")
    add_launch_arg("num_points_thresholds", "300")
    add_launch_arg("invalid_intensity")
    add_launch_arg("invalid_point_remove", "False")
    add_launch_arg("invalid_rings")
    add_launch_arg("invalid_angles_start")
    add_launch_arg("invalid_angles_end")
    add_launch_arg("frame_id", "velodyne", "velodyne frame id")
    add_launch_arg("gps_time", "False")
    add_launch_arg("view_direction", description="the center of lidar angle")
    add_launch_arg("view_width", description="lidar angle: 0~6.28 [rad]")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg(
        "vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml"
    )
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS2 component container communication")
    add_launch_arg("use_pointcloud_container", "false")
    add_launch_arg("container_name", "velodyne_node_container")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
