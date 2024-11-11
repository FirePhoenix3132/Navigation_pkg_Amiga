from __future__ import annotations

import argparse
import asyncio
from pathlib import Path
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.gps import gps_pb2


def publish_gps_frame(msg, gps_pub):
    # Create a NavSatFix message
    gps_msg = NavSatFix()

    # Fill the GPS data
    gps_msg.header.stamp = rospy.Time.now()
    gps_msg.status.status = NavSatStatus.STATUS_FIX if msg.gnss_fix_ok else NavSatStatus.STATUS_NO_FIX
    gps_msg.status.service = NavSatStatus.SERVICE_GPS
    gps_msg.latitude = msg.latitude
    gps_msg.longitude = msg.longitude
    gps_msg.altitude = msg.altitude
    gps_msg.position_covariance[0] = msg.horizontal_accuracy ** 2  # Variance as square of accuracy
    gps_msg.position_covariance[4] = msg.horizontal_accuracy ** 2  # Variance for latitude and longitude
    gps_msg.position_covariance[8] = msg.vertical_accuracy ** 2    # Variance for altitude
    gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

    # Publish the message
    gps_pub.publish(gps_msg)


def print_relative_position_frame(msg):
    print(f"Message stamp: {msg.stamp.stamp}")
    print(f"GPS time: {msg.gps_time.stamp}")
    print(f"Relative pose north: {msg.relative_pose_north}")
    print(f"Relative pose east: {msg.relative_pose_east}")
    print(f"Relative pose down: {msg.relative_pose_down}")
    print(f"Relative pose length: {msg.relative_pose_length}")
    print(f"Accuracy north: {msg.accuracy_north}")
    print(f"Accuracy east: {msg.accuracy_east}")
    print(f"Accuracy down: {msg.accuracy_down}")
    print(f"Carrier solution: {msg.carr_soln}")
    print(f"GNSS fix ok: {msg.gnss_fix_ok}")
    print("-" * 50)


def print_gps_frame(msg):
    print(f"Message stamp: {msg.stamp.stamp}")
    print(f"GPS time: {msg.gps_time.stamp}")
    print(f"Latitude: {msg.latitude}")
    print(f"Longitude: {msg.longitude}")
    print(f"Altitude: {msg.altitude}")
    print(f"Ground speed: {msg.ground_speed}")
    print(f"Speed accuracy: {msg.speed_accuracy}")
    print(f"Horizontal accuracy: {msg.horizontal_accuracy}")
    print(f"Vertical accuracy: {msg.vertical_accuracy}")
    print(f"P DOP: {msg.p_dop}")
    print("-" * 50)


async def main(service_config_path: Path) -> None:
    """Run the gps service client.

    Args:
        service_config_path (Path): The path to the gps service config.
    """
    # Initialize ROS node and publisher
    rospy.init_node("amiga_gps_stream")
    gps_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
    print('Started GPS....')
    # Create a client to the GPS service
    config: EventServiceConfig = proto_from_json_file(service_config_path, EventServiceConfig())
    async for event, msg in EventClient(config).subscribe(config.subscriptions[0]):

        if isinstance(msg, gps_pb2.RelativePositionFrame):
            print_relative_position_frame(msg)
        elif isinstance(msg, gps_pb2.GpsFrame):
            print_gps_frame(msg)
            publish_gps_frame(msg, gps_pub)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="amiga-gps-stream")
    parser.add_argument("--service-config", type=Path, required=True, help="The GPS config.")
    args = parser.parse_args()

    asyncio.run(main(args.service_config))
