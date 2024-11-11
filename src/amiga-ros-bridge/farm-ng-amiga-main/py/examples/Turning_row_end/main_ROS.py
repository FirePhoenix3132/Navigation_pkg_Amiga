"""Example using the track_follower service to drive a square."""
# Copyright (c) farm-ng, inc.
#
# Licensed under the Amiga Development Kit License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://github.com/farm-ng/amiga-dev-kit/blob/main/LICENSE
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations

import rospy
from std_msgs.msg import Bool

import argparse
import asyncio
from math import copysign
from math import radians
from pathlib import Path
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path



from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfigList
from farm_ng.core.event_service_pb2 import SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.core.uri_pb2 import Uri
from farm_ng.filter.filter_pb2 import FilterState
from farm_ng.track.track_pb2 import Track
from farm_ng.track.track_pb2 import TrackFollowerState
from farm_ng.track.track_pb2 import TrackFollowRequest
from farm_ng_core_pybind import Isometry3F64
from farm_ng_core_pybind import Pose3F64
from farm_ng_core_pybind import Rotation3F64
from google.protobuf.empty_pb2 import Empty
from track_planner import TrackBuilder

turn_end_flag = False  # Global flag to track the turn end status

async def get_pose(clients: dict[str, EventClient]) -> Pose3F64:
    """Get the current pose of the robot in the world frame, from the filter service.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
    """
    # We use the FilterState as the best source of the current pose of the robot
    state: FilterState = await clients["filter"].request_reply("/get_state", Empty(), decode=True)
    print(f"Current filter state:\n{state}")
    return Pose3F64.from_proto(state.pose)


async def set_track(clients: dict[str, EventClient], track: Track) -> None:
    """Set the track of the track_follower.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
        track (Track): The track for the track_follower to follow.
    """
    print(f"Setting track:\n{track}")
    await clients["track_follower"].request_reply("/set_track", TrackFollowRequest(track=track))


async def start(clients: dict[str, EventClient]) -> None:
    """Request to start following the track.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
    """
    print("Sending request to start following the track...")
    await clients["track_follower"].request_reply("/start", Empty())


async def build_square(clients: dict[str, EventClient], side_length: float, clockwise: bool) -> Track:
    """Build a square track, from the current pose of the robot.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
        side_length (float): The side length of the square, in meters.
        clockwise (bool): True will drive the square clockwise (right hand turns).
                        False is counter-clockwise (left hand turns).
    Returns:
        Track: The track for the track_follower to follow.
    """

    # Query the state estimation filter for the current pose of the robot in the world frame
    world_pose_robot: Pose3F64 = await get_pose(clients)

    # Create a container to store the track waypoints
    track_waypoints: list[Pose3F64] = []

    # Set the angle of the turns, based on indicated direction
    angle: float = radians(-90) if clockwise else radians(90)

    # Add the first goal at the current pose of the robot
    world_pose_goal0: Pose3F64 = world_pose_robot * Pose3F64(a_from_b=Isometry3F64(), frame_a="robot", frame_b="goal0")
    track_waypoints.append(world_pose_goal0)

    # Drive forward 1 meter (first side of the square)
    track_waypoints.extend(create_straight_segment(track_waypoints[-1], "goal1", side_length))

    # Turn left 90 degrees (first turn)
    track_waypoints.extend(create_turn_segment(track_waypoints[-1], "goal2", angle))

    # Add second side and turn
    track_waypoints.extend(create_straight_segment(track_waypoints[-1], "goal3", side_length))
    track_waypoints.extend(create_turn_segment(track_waypoints[-1], "goal4", angle))

    # Add third side and turn
    track_waypoints.extend(create_straight_segment(track_waypoints[-1], "goal5", side_length))
    track_waypoints.extend(create_turn_segment(track_waypoints[-1], "goal6", angle))

    # Add fourth side and turn
    track_waypoints.extend(create_straight_segment(track_waypoints[-1], "goal7", side_length))
    track_waypoints.extend(create_turn_segment(track_waypoints[-1], "goal8", angle))

    # Return the list of waypoints as a Track proto message
    return format_track(track_waypoints)

async def create_start_pose(client: EventClient | None = None, timeout: float = 0.5) -> Pose3F64:
    """Create a start pose for the track.

    Args:
        client: A EventClient for the required service (filter)
    Returns:
        The start pose (Pose3F64)
    """
    print("Creating start pose...")

    zero_tangent = np.zeros((6, 1), dtype=np.float64)
    start: Pose3F64 = Pose3F64(
        a_from_b=Isometry3F64(), frame_a="world", frame_b="robot", tangent_of_b_in_a=zero_tangent
    )
    if client is not None:
        try:
            # Get the current state of the filter
            state: FilterState = await asyncio.wait_for(
                client.request_reply("/get_state", Empty(), decode=True), timeout=timeout
            )
            start = Pose3F64.from_proto(state.pose)
        except asyncio.TimeoutError:
            print("Timeout while getting filter state. Using default start pose.")
        except Exception as e:
            print(f"Error getting filter state: {e}. Using default start pose.")

    return start

async def build_track(save_track_flag: bool, clients: dict[str, EventClient] | None = None, save_track: Path | None = None) -> Track:
    """Builds a custom track for the Amiga to follow.

    Args:
        reverse: Whether or not to reverse the track
        client: A EventClient for the required service (filter)
        save_track: The path to save the track to
    Returns:
        The track
    """
    print("Building track...")

    row_spacing: float = 36 * 0.0254  # 36 inches in meters
    extended_length: float = 3 * 12 * 0.0254  # 3 feet in meters
    row_length: float = extended_length
    turning_angle = 180  #neg for right turn and pos for left turn

    # Path: Start at the beginning row 2, go up on 2, down on row 4, up on row 1, down on row 3, up on row 1,
    # down on row 4, and finish lining up on row 2
    # Assumption: At run time, the robot is positioned at the beginning of row 2, facing the end of row 2.

    # start: Pose3F64 = await create_start_pose(clients)
    start: Pose3F64 = await get_pose(clients)

    track_builder = TrackBuilder(start=start)

    # Drive forward 32 ft (up row 2)
    track_builder.create_straight_segment(next_frame_b="goal1", distance=row_length, spacing=0.1)

    # Maneuver at the end of row: skip one row (72 inches) - (go from 2 to 4)
    track_builder.create_arc_segment(next_frame_b="goal2", radius=row_spacing, angle=radians(turning_angle), spacing=0.1)

    # Drive forward 32 ft (down 4)
    track_builder.create_straight_segment(next_frame_b="goal3", distance=row_length, spacing=0.1)

    # # Maneuver at the end of row: skip two rows (144 inches) - (go from 4 to 1)
    # track_builder.create_arc_segment(next_frame_b="goal4", radius=1.5 * row_spacing, angle=radians(180), spacing=0.1)

    # # Drive forward 32 ft (up 1)
    # track_builder.create_straight_segment(next_frame_b="goal5", distance=row_length, spacing=0.1)

    # # Maneuver at the end of row: skip one row (96 inches) - (go from 1 to 3)
    # track_builder.create_arc_segment(next_frame_b="goal6", radius=row_spacing, angle=radians(180), spacing=0.1)

    # # Drive forward 32 ft (down 3)
    # track_builder.create_straight_segment(next_frame_b="goal7", distance=row_length, spacing=0.1)

    # # Maneuver at the end of row: skip one row (96 inches) - (go from 3 to 1)
    # track_builder.create_arc_segment(next_frame_b="goal8", radius=row_spacing, angle=radians(180), spacing=0.1)

    # # Drive forward 32 ft (up 1)
    # track_builder.create_straight_segment(next_frame_b="goal9", distance=row_length, spacing=0.1)

    # # Maneuver at the end of row: skip two rows (144 inches) - (go from 1 to 4)
    # track_builder.create_arc_segment(next_frame_b="goal10", radius=1.5 * row_spacing, angle=radians(180), spacing=0.1)

    # # Drive forward 32 ft (down 4)
    # track_builder.create_straight_segment(next_frame_b="goal11", distance=row_length, spacing=0.1)

    # # Maneuver at the end of row: skip one row (96 inches) - (go from 4 to 2 - slightly before the start)
    # track_builder.create_arc_segment(next_frame_b="goal12", radius=row_spacing, angle=radians(175), spacing=0.1)


    # Print the number of waypoints in the track
    print(f" Track created with {len(track_builder.track_waypoints)} waypoints")

    # Save the track to a file
    if save_track is not None:
        track_builder.save_track(save_track)

    # Plot the track
    waypoints = track_builder.unpack_track()
    plot_track(waypoints)
    return track_builder.track

def plot_track(waypoints: list[list[float]]) -> None:
    x = waypoints[0]
    y = waypoints[1]
    headings = waypoints[2]

    # Calculate the arrow directions
    U = np.cos(headings)
    V = np.sin(headings)

    # Parameters for arrow plotting
    arrow_interval = 20  # Adjust this to change the frequency of arrows
    turn_threshold = np.radians(10)  # Threshold in radians for when to skip plotting

    plt.figure(figsize=(8, 8))
    plt.plot(x, y, color='orange', linewidth=1.0)

    for i in range(0, len(x), arrow_interval):
        # Calculate the heading change
        if i > 0:
            heading_change = np.abs(headings[i] - headings[i - 1])
        else:
            heading_change = 0

        # Plot the arrow if the heading change is below the threshold
        if heading_change < turn_threshold:
            plt.quiver(x[i], y[i], U[i], V[i], angles='xy', scale_units='xy', scale=3.5, color='blue')

    plt.plot(x[0], y[0], marker="o", markersize=5, color='red')
    plt.axis("equal")
    legend_elements = [
        plt.Line2D([0], [0], color='orange', lw=2, label='Track'),
        plt.Line2D([0], [0], color='blue', lw=2, label='Heading'),
        plt.scatter([], [], color='red', marker='o', s=30, label='Start'),
    ]
    plt.legend(handles=legend_elements)
    plt.show()

def create_straight_segment(
    previous_pose: Pose3F64, next_frame_b: str, distance: float, spacing: float = 0.1
) -> list[Pose3F64]:
    """Compute a straight segment of a square.

    Args:
        previous_pose (Pose3F64): The previous pose.
        next_frame_b (str): The name of the child frame of the next pose.
        distance (float): The side length of the square, in meters.
        spacing (float): The spacing between waypoints, in meters.

    Returns:
        Pose3F64: The poses of the straight segment.
    """
    # Create a container to store the track segment waypoints
    segment_poses: list[Pose3F64] = [previous_pose]

    # For tracking the number of segments and remaining angle
    counter: int = 0
    remaining_distance: float = distance

    while abs(remaining_distance) > 0.01:
        # Compute the distance of the next segment
        segment_distance: float = copysign(min(abs(remaining_distance), spacing), distance)

        # Compute the next pose
        straight_segment: Pose3F64 = Pose3F64(
            a_from_b=Isometry3F64([segment_distance, 0, 0], Rotation3F64.Rz(0)),
            frame_a=segment_poses[-1].frame_b,
            frame_b=f"{next_frame_b}_{counter}",
        )
        segment_poses.append(segment_poses[-1] * straight_segment)

        # Update the counter and remaining angle
        counter += 1
        remaining_distance -= segment_distance

    # Rename the last pose to the desired name
    segment_poses[-1].frame_b = next_frame_b
    return segment_poses


def create_turn_segment(
    previous_pose: Pose3F64, next_frame_b: str, angle: float, spacing: float = 0.1
) -> list[Pose3F64]:
    """Compute a turn segment of a square.

    Args:
        previous_pose (Pose3F64): The previous pose.
        next_frame_b (str): The name of the child frame of the next pose.
        angle (float): The angle to turn, in radians (+ left, - right).
        spacing (float): The spacing between waypoints, in radians.
    Returns:
        list[Pose3F64]: The poses of the turn segment.
    """
    # Create a container to store the track segment waypoints
    segment_poses: list[Pose3F64] = [previous_pose]

    # For tracking the number of segments and remaining angle
    counter: int = 0
    remaining_angle: float = angle

    while abs(remaining_angle) > 0.01:
        # Compute the angle of the next segment
        segment_angle: float = copysign(min(abs(remaining_angle), spacing), angle)

        # Compute the next pose
        turn_segment: Pose3F64 = Pose3F64(
            a_from_b=Isometry3F64.Rz(segment_angle),
            frame_a=segment_poses[-1].frame_b,
            frame_b=f"{next_frame_b}_{counter}",
        )
        segment_poses.append(segment_poses[-1] * turn_segment)

        # Update the counter and remaining angle
        counter += 1
        remaining_angle -= segment_angle

    # Rename the last pose to the desired name
    segment_poses[-1].frame_b = next_frame_b
    return segment_poses


def format_track(track_waypoints: list[Pose3F64]) -> Track:
    """Pack the track waypoints into a Track proto message.

    Args:
        track_waypoints (list[Pose3F64]): The track waypoints.
    """
    return Track(waypoints=[pose.to_proto() for pose in track_waypoints])


async def start_track(clients: dict[str, EventClient],save_track_flag: bool, save_track: Path | None = None) -> None:
    """Run the track_follower square example. The robot will drive a square, turning left at each corner.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
        side_length (float): The side length of the square.
        clockwise (bool): True will drive the square clockwise (right hand turns).
                        False is counter-clockwise (left hand turns).
    """

    # Build the track and package in a Track proto message
    track: Track = await build_track(save_track_flag, clients, save_track)

    # Send the track to the track_follower
    await set_track(clients, track)

    # Start following the track
    await start(clients)


async def stream_track_state(clients: dict[str, EventClient]) -> None:
    """Stream the track_follower state.

    Args:
        clients (dict[str, EventClient]): A dictionary of EventClients.
    """

    # Brief wait to allow you to see the track sent to the track_follower
    # Note that this is not necessary in practice
    await asyncio.sleep(1.0)

    # Subscribe to the track_follower state and print each
    message: TrackFollowerState
    async for _, message in clients["track_follower"].subscribe(SubscribeRequest(uri=Uri(path="/state"))):
        print("###################")
        print(message)



def turn_end_callback(msg: Bool):
    """ROS callback to set turn_end_flag based on incoming messages."""
    global turn_end_flag
    if msg.data:
        turn_end_flag = True
        rospy.loginfo("Turn end flag received as True. Track creation will initiate.")

def setup_turn_end_subscriber():
    """Setup ROS subscriber for turn_end_flag."""
    rospy.Subscriber("/turn_end_flag", Bool, turn_end_callback)

async def start_track_on_flag(clients: dict[str, EventClient], save_track_flag: bool, save_track: Path | None = None) -> None:
    """Start track creation only when turn_end_flag becomes True."""
    global turn_end_flag
    rospy.loginfo("Waiting for turn_end_flag to be True...")
    while not turn_end_flag and not rospy.is_shutdown():
        rospy.sleep(0.5)  # Poll every 0.5 seconds

        if turn_end_flag:
            rospy.loginfo("Turn end flag is True. Starting track creation...")
            # start_track(clients, save_track_flag, save_track)
                # Start the asyncio tasks
            tasks: list[asyncio.Task] = [
                asyncio.create_task(start_track(clients,save_track_flag, args.save_track)),
                asyncio.create_task(stream_track_state(clients)),
            ]
            await asyncio.gather(*tasks)

async def run(args) -> None:
    """Main function to initialize the ROS node, EventClients, and start track creation."""
    global turn_end_flag
    # Initialize the ROS node
    rospy.init_node("track_creation_node", anonymous=True)

    # Set up save track flag
    save_track_flag: bool = args.save_track
    args.save_track = Path('~/catkin_ws_amiga/src/amiga-ros-bridge/farm-ng-amiga-main/py/examples/Turning_row_end/')
    # Create EventClients for required services
    clients: dict[str, EventClient] = {}
    expected_configs = ["track_follower", "filter"]
    config_list = proto_from_json_file(args.service_config, EventServiceConfigList())
    for config in config_list.configs:
        if config.name in expected_configs:
            clients[config.name] = EventClient(config)

    # Check all required services were initialized
    for config in expected_configs:
        if config not in clients:
            raise RuntimeError(f"No {config} service config in {args.service_config}")

    # Set up the turn end flag subscriber
    setup_turn_end_subscriber()

    # Start the track creation task
    # start_track_on_flag(clients, save_track_flag, save_track=None)   #Test this and check whether its working or not. If it works use this and comment out the below code.


    while not turn_end_flag and not rospy.is_shutdown():
        rospy.sleep(0.5)  # Poll every 0.5 seconds

        if turn_end_flag:
            rospy.loginfo("Turn end flag is True. Starting track creation...")
            # start_track(clients, save_track_flag, save_track)
                # Start the asyncio tasks
            tasks: list[asyncio.Task] = [
                asyncio.create_task(start_track(clients,save_track_flag, args.save_track)),
                asyncio.create_task(stream_track_state(clients)),
            ]
            await asyncio.gather(*tasks)

    # if turn_end_flag:
    #     rospy.loginfo("Turn end flag is True. Starting track creation in 5 secs...")
    #     # start_track(clients, save_track_flag, save_track)
    #         # Start the asyncio tasks
    #     tasks: list[asyncio.Task] = [
    #         asyncio.create_task(start_track(clients,save_track_flag, args.save_track)),
    #         asyncio.create_task(stream_track_state(clients)),
    #     ]
    #     await asyncio.gather(*tasks)

    # Keep the node alive
    rospy.spin()


# async def run(args) -> None:
#     # Create flag for saving track
#     save_track_flag: bool = args.save_track #If path is given, then its true. For simplicity you can change this to True or False.
#     # Create a dictionary of EventClients to the services required by this example
#     clients: dict[str, EventClient] = {}
#     expected_configs = ["track_follower", "filter"]
#     config_list = proto_from_json_file(args.service_config, EventServiceConfigList())
#     for config in config_list.configs:
#         if config.name in expected_configs:
#             clients[config.name] = EventClient(config)

#     # Confirm that EventClients were created for all required services
#     for config in expected_configs:
#         if config not in clients:
#             raise RuntimeError(f"No {config} service config in {args.service_config}")

#     # Start the asyncio tasks
#     tasks: list[asyncio.Task] = [
#         asyncio.create_task(start_track(clients,save_track_flag, args.save_track)),
#         asyncio.create_task(stream_track_state(clients)),
#     ]
#     await asyncio.gather(*tasks)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="python main.py", description="Amiga track_follower turning.")
    parser.add_argument("--service-config", type=Path, required=True, help="The service config.")
    # parser.add_argument("--side-length", type=float, default=2.0, help="The side length of the square.")
    parser.add_argument("--save-track", type=Path, help="Save the track to a file.")
    # parser.add_argument("--reverse", action='store_true', help="Reverse the track.")
    # parser.add_argument(
    #     "--clockwise",
    #     action="store_true",
    #     help="Set to drive the square clockwise (right hand turns). Default is counter-clockwise (left hand turns).",
    # )
    args = parser.parse_args()

    # Create the asyncio event loop and run the main function
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args))
