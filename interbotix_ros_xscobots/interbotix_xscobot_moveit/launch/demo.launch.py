from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("dx400", package_name="interbotix_xscobot_moveit").to_moveit_configs()
    return generate_demo_launch(moveit_config)
