from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    """ This function starts the system in a custom configuration """

    # Generate the object that must be returned by this function
    ld = LaunchDescription()

    project_name='offboard_control'

    ##################################################
    #                 Mount the nodes                
    ##################################################

    offboard_control_node = Node(
        package=project_name,
        executable="star_movement",
    )

    ##################################################
    #   Add the action to LaunchDescription object 
    ##################################################

    ld.add_action(offboard_control_node)

    return ld