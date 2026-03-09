from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time = True)

sl_trajectory = sl.declare_arg('trajectory', default_value = 'station_keeping')
sl_spawn = sl.declare_arg('spawn_pose', default_value='0.0 0.0 0.0 0.0 0.0 0.0')
sl_robot = sl.declare_arg('robot_file', default_value='thrusters')

def launch_setup():
    sl.include('blueboat_description', 'world_launch.py', launch_arguments={'sliders': False, 'spawn_pose': sl_spawn, 'thr': sl_robot})

    sl.node('blueboat_control', 'path_generation.py', parameters={'trajectory' : sl_trajectory})

    sl.node('blueboat_control', 'path_publisher.py')

    sl.node('blueboat_control', 'ur_mpc_control.py')

    return sl.launch_description()

generate_launch_description = sl.launch_description(opaque_function = launch_setup)
