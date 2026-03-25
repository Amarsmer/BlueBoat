from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time = True)

sl_trajectory = sl.declare_arg('trajectory', default_value = 'station_keeping')
sl_spawn = sl.declare_arg('spawn_pose', default_value='0.0 0.0 0.0')
sl_motors = sl.declare_arg('enable_motors', default_value = False)

def launch_setup():
    # sl.include('blueboat_description', 'world_launch.py', launch_arguments={'sliders': False, 'spawn_pose': sl_spawn})
    
    sl.node('mavros',        # package
            'mavros_node',   # executable
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14550@192.168.2.2:14550',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1
            }])

    sl.node('blueboat_control', 'interface.py', parameters={'enable_motors' : sl_motors,
                                                            'use_sim_time': False})
                                                        
    sl.node('blueboat_control', 'param_set.py')

    # sl.node('blueboat_control', 'path_generation.py', parameters={'trajectory' : sl_trajectory})
    
    sl.node('blueboat_control', 'uwgps_log.py', parameters={'use_sim_time': False})

    # sl.node('blueboat_control', 'ur_mpc_control.py')

    return sl.launch_description()

generate_launch_description = sl.launch_description(opaque_function = launch_setup)
