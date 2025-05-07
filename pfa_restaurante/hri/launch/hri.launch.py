# Copyright 2023 Rodrigo Pérez-Rodríguez
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('hri_bt_nodes')
    
    params_file = os.path.join(pkg_dir, 'config', 'hri.yaml')
    # print('params_file: ', params_file)
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['hri_node']['ros__parameters']
    # print(params)

    ld = LaunchDescription()

    hri_cmd = Node(
        package='hri_bt_nodes',
        executable='hri_test',
        output='screen',
        remappings=[
        ],
        parameters=[{
            'use_sim_time': True,
        }, params]
    )

    ld.add_action(hri_cmd)

    return ld

