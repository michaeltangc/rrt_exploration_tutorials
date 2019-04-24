#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse

maps = [
    """<arg name="world_name" value="$(find exercises)/part1/ros/worlds/simple.world"/>""",
    """<arg name="world_name" value="$(find rrt_exploration_tutorials)/launch/includes/worlds/MTR.world"/>""",
    """<arg name="world_name" value="$(find rrt_exploration_tutorials)/launch/includes/worlds/house.world"/>""",
    """<arg name="world_name" value="$(find rrt_exploration_tutorials)/launch/includes/worlds/largeMap.world"/>"""
]

sections = {
0:"""
<launch>
    <env name="GAZEBO_RESOURCE_PATH" value="$(find rrt_exploration_tutorials)/launch/includes/meshes"/>
    <!-- start Gazebo with an empty world -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="use_sim_time" value="true"/>
        <arg name="debug" value="false"/>

""",
1:"""

    </include>

    <arg name="eta" value="1.0"/>
    <arg name="Geta" value="1.0"/>

""",
2:"""
    <group ns="/{robot_name:s}">
        <include file="$(find rrt_exploration_tutorials)/launch/includes/robot.launch.xml">
            <arg name="robot_name" value="{robot_name:s}"/>
            <arg name="init_pose" value="-x {x:.1f} -y {y:.1f} -z {z:.1f}"/>
        </include>
        <include file="$(find rrt_exploration_tutorials)/launch/includes/move_baseSafe.launch">
            <arg name="namespace" value="{robot_name:s}"/>
        </include> 
    </group>
""",
3:"""
    <node pkg="tf" type="static_transform_publisher" name="{robot_name:s}_to_robot_1" args="{x:.1f} {y:.1f} {z:.1f} 0 0 0 /robot_1/map /{robot_name:s}/map 20" />
""",
4:"""
    <group ns="/{robot_name:s}/map_merge">
        <param name="init_pose_x" value="{x:.1f}"/>
        <param name="init_pose_y" value="{y:.1f}"/>
        <param name="init_pose_z" value="{z:.1f}"/>
        <param name="init_pose_yaw" value="0.0"/>
    </group>
""",
5:"""

    <node pkg="rrt_exploration" type="global_rrt_detector" name="{robot_name:s}_global_rrt_detector" output="screen">
        <param name="eta" value="$(arg Geta)"/>
        <param name="map_topic" value="/map_merge/map"/>

        <param name="detected_points_topic" value="/{robot_name:s}/detected_points"/> 
    </node>

    <node pkg="rrt_exploration" type="local_rrt_detector" name="{robot_name:s}_rrt_detector" output="screen">
        <param name="eta" value="$(arg eta)"/>
        <param name="map_topic" value="/{robot_name:s}/map"/>
        <param name="robot_frame" value="/{robot_name:s}/base_link"/>

        <param name="detected_points_topic" value="/{robot_name:s}/detected_points"/> 
    </node>
""",
6:"""
    <group ns="/robot_1">
        <node pkg="rrt_exploration" type="filter.py" name="{robot_name:s}_filter" output="screen">
            <param name="map_topic" value="/map_merge/map"/>
            <param name="info_radius" value="1"/> 
            <param name="costmap_clearing_threshold" value="70"/> 
            <param name="goals_topic" value="/{robot_name:s}/detected_points"/> 
            <param name="n_robots" value="{num_robot:d}"/>
            <param name="namespace_init_count" value="{robot_id:d}"/>
            <param name="namespace" value="/robot_"/>
            <param name="rate" value="100"/>
        </node>
    </group>
        
        
    <node pkg="rrt_exploration" type="assigner.py" name="{robot_name:s}_assigner" output="screen">
        <param name="map_topic" value="/map_merge/map"/>
        <param name="global_frame" value="/{robot_name:s}/map"/>
        <param name="info_radius" value="1"/> 
        <param name="info_multiplier" value="3.0"/> 
        <param name="hysteresis_radius" value="3.0"/> 
        <param name="hysteresis_gain" value="2.0"/> 
        <param name="frontiers_topic" value="/{robot_name:s}/filtered_points"/> 
        <param name="n_robots" value="{num_robot:d}"/>
        <param name="namespace_init_count" value="1{robot_id:d}"/>
        <param name="namespace" value="/robot_"/>
        <param name="delay_after_assignement" value="0.5"/>
        <param name="rate" value="100"/>
    </node>
""",
7:"""
    <!-- <include file="$(find rrt_exploration_tutorials)/launch/includes/map_merge.launch"/> -->

    <group ns="map_merge">
        <node pkg="multirobot_map_merge" type="map_merge" respawn="false" name="map_merge" output="screen">
            <param name="robot_map_topic" value="map"/>
            <param name="robot_namespace" value=""/>
            <param name="merged_map_topic" value="map"/>
            <param name="world_frame" value="/robot_1/map"/>
            <param name="known_init_poses" value="true"/>
            <param name="merging_rate" value="10.0"/>
            <param name="discovery_rate" value="0.05"/>
            <param name="estimation_rate" value="0.5"/>
            <param name="estimation_confidence" value="1.0"/>
        </node>
    </group>

    <!-- run RViz node (visualization) -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find rrt_exploration_tutorials)/launch/includes/rviz_config/three_decentralised.rviz">
        <remap from="move_base_simple/goal" to="robot_1/move_base_simple/goal"/>
    </node>

</launch>

"""
}

def generate_file(num_robot, file_name, map_index):
    num_robot = int(num_robot)
    map_index = int(map_index)
    file_name = 'decentralised_' + str(num_robot) + '_' + str(map_index) + '_run.launch' if file_name is None else file_name
    with open(file_name, 'w+') as f:
        f.write(sections[0])
        f.write(maps[map_index])
        f.write(sections[1])
        for i in range(num_robot):
            robot_id = i+1
            robot_name = "robot_{:d}".format(robot_id)
            x = 0.0
            y = 1.0 * ((i+1) // 2) * (-1 if i % 2 == 1 else 1)
            z = 0.0
            
            f.write(sections[2].format(robot_name=robot_name, x=x, y=y, z=z))
            if not i == 0:
                f.write(sections[3].format(robot_name=robot_name, x=x, y=y, z=z))
            f.write(sections[4].format(robot_name=robot_name, x=x, y=y, z=z))
            f.write(sections[5].format(robot_name=robot_name))
            f.write(sections[6].format(robot_name=robot_name, num_robot=1, robot_id=robot_id))
        f.write(sections[7])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate run file')
    parser.add_argument('--num_robot', action='store', default=1)
    parser.add_argument('--file_name', action='store', default=None)
    parser.add_argument('--map_index', action='store', default=0)
    args, unknown = parser.parse_known_args()
    generate_file(args.num_robot, args.file_name, args.map_index)


