from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      Node(
         package='camera_net',
         executable='client',
         name='client',
         parameters=[{'camera_info_url': '/home/gcc/ros2_ws/ost.yaml'}] 
      ),
      Node(
         package='image_proc',
         executable='rectify_node',
         name='image_proc',
         remappings=[
             ('image', 'image_raw'),
             ('image_rect', 'image_rect')
         ],
      ),
   ])
