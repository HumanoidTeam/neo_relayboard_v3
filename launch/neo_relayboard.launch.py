"""Launch neo_relayboard_v3 node and watchdog with rox config (same resolution as alpha_base_bringup)."""

import pathlib

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


RELAYBOARD_NODE_NAME = "base_controller_node"


def generate_launch_description() -> LaunchDescription:
    # Rox config: use source tree (vnx middleware doesn't support symlinks).
    repo_root = pathlib.Path(__file__).resolve().parents[4]
    rox_share_path = repo_root / "ros" / "third_party" / "rox" / "rox_bringup"
    if not rox_share_path.exists():
        rox_share_path = get_package_share_path("rox_bringup").resolve()
    rox_config = str(rox_share_path / "configs" / "neo_relayboard_v3" / "rox-argo")

    relayboard_node = Node(
        package="neo_relayboard_v3",
        executable="relayboardv3_node",
        output="screen",
        name=RELAYBOARD_NODE_NAME,
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[{"pilot_config": rox_config}],
        respawn=True,
        respawn_delay=1.0,
    )

    watchdog_node = Node(
        package="neo_relayboard_v3",
        executable="relayboard_watchdog",
        name="relayboard_watchdog",
        output="screen",
        parameters=[
            {
                "state_topic": "/relayboard_v3/state",
                "startup_timeout_sec": 5.0,
                "deadman_timeout_sec": 2.0,
                "target_node_name": f"/{RELAYBOARD_NODE_NAME}",
            }
        ],
    )

    return LaunchDescription([relayboard_node, watchdog_node])
