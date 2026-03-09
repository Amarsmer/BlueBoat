from simple_launch import SimpleLauncher


def generate_launch_description():

        sl = SimpleLauncher(use_sim_time = True)

        sl_robot = sl.declare_arg('robot_file', default_value='thrusters_plasmar2')
        sl.include('blueboat_description', 'world_launch.py', launch_arguments={'sliders': False, 'thr': sl_robot})

        with sl.group(ns = 'blueboat'):

                # load body controller anyway
                sl.node('auv_control', 'cascaded_pid',
                parameters=[sl.find('blueboat_control', 'cascaded_pid.yaml')],
                output='screen')

                sl.node('slider_publisher', 'slider_publisher', name='pose_control',
                arguments=[sl.find('auv_control', 'pose_setpoint.yaml')])

                # sl.node('slider_publisher', 'slider_publisher', name='joint_control',
                #         arguments=[sl.find('blueboat_control', 'thrusters_plasmar2_joints.yaml')])

                sl.node('thruster_manager', 'publish_wrenches',
                parameters={'control_frame': 'blueboat/base_link',
                        'use_gz_topics': sl.sim_time})

        return sl.launch_description()
