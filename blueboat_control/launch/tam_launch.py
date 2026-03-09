from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher(use_sim_time = True)

    with sl.group(ns = 'blueboat'):

        sl.node('thruster_manager', 'thruster_manager_node',
                parameters = {'control_frame': 'blueboat/base_link'})




    return sl.launch_description()




