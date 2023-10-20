import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('inverse_kinematics'),
      'config',
      'IK_parameters.yaml'
      )

   return LaunchDescription([
      Node(
         package='inverse_kinematics',
         executable='inverse_kinematics_basic',
         namespace='',
         name='inverse_kinematics_basic',
         parameters=[config]
      )
   ])
