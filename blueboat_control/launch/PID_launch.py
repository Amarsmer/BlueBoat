from simple_launch import SimpleLauncher


def generate_launch_description():
        sl = SimpleLauncher(use_sim_time = True)

        sl_robot = sl.declare_arg('robot_file', default_value='thrusters_ur')
        sl_trajectory = sl.declare_arg('trajectory', default_value = 'station_keeping')

        sl.include('blueboat_description', 'world_launch.py', launch_arguments={'sliders': False, 'thr': sl_robot})

        sl.node('blueboat_control', 'PID_path.py')

        sl.node('blueboat_control', 'path_generation.py', parameters={'trajectory' : sl_trajectory})

        sl.node('blueboat_control', 'path_publisher.py')

        with sl.group(ns = 'blueboat'):

                # load body controller anyway
                sl.node('auv_control', 'cascaded_pid',
                parameters=[sl.find('blueboat_control', 'cascaded_pid.yaml')],
                output='screen')

                # sl.node('slider_publisher', 'slider_publisher', name='pose_control',
                # arguments=[sl.find('auv_control', 'pose_setpoint.yaml')])

                # sl.node('thruster_manager', 'publish_wrenches',
                # parameters={'control_frame': 'blueboat/base_link',
                #         'use_gz_topics': sl.sim_time})

        return sl.launch_description()
