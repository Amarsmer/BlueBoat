from simple_launch import SimpleLauncher


def generate_launch_description():

        sl = SimpleLauncher(use_sim_time = True)
        sl.include('blueboat_description', 'world_launch.py', launch_arguments={'sliders': False})

        with sl.group(ns = 'blueboat'):

                # load body controller anyway
                sl.node('auv_control', 'cascaded_pid',
                parameters=[sl.find('blueboat_control', 'cascaded_pid.yaml')],
                output='screen')

                sl.node('slider_publisher', 'slider_publisher', name='pose_control',
                arguments=[sl.find('auv_control', 'pose_setpoint.yaml')])

                sl.node('thruster_manager', 'publish_wrenches',
                parameters={'control_frame': 'blueboat/base_link',
                        'use_gz_topics': sl.sim_time})

        return sl.launch_description()
