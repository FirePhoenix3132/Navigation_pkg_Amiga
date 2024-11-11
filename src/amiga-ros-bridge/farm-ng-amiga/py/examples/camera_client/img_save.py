from __future__ import annotations
import rospy
import argparse
import asyncio
from pathlib import Path

import cv2
import numpy as np
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.core.stamp import get_stamp_by_semantics_and_clock_type
from farm_ng.core.stamp import StampSemantics

from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge     # Package for OpenCV-ROS image conversion

async def main(service_config_path: Path) -> None:
    """Run the camera service client.

    Args:
        service_config_path (Path): The path to the camera service config.
    """
    print("started async")

    # Initialize ROS node and publisher
    rospy.init_node("amiga_camera_stream")
    image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
    
    bridge = CvBridge()  # Create CvBridge object to convert images

    # create a client to the camera service
    config: EventServiceConfig = proto_from_json_file(service_config_path, EventServiceConfig())

    async for event, message in EventClient(config).subscribe(config.subscriptions[0], decode=True):
        # Find the monotonic driver receive timestamp, or the first timestamp if not available.
        stamp = (
            get_stamp_by_semantics_and_clock_type(event, StampSemantics.DRIVER_RECEIVE, "monotonic")
            or event.timestamps[0].stamp
        )

        # print the timestamp and metadata
        print(f"Timestamp: {stamp}\n")
        print(f"Meta: {message.meta}")
        print("###################\n")

        # cast image data bytes to numpy and decode
        image = cv2.imdecode(np.frombuffer(message.image_data, dtype="uint8"), cv2.IMREAD_UNCHANGED)
        if event.uri.path == "/disparity":
            image = cv2.applyColorMap(image * 3, cv2.COLORMAP_JET)

        # Convert OpenCV image to ROS Image message and publish it
        ros_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_pub.publish(ros_image)

        # visualize the image
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.imshow("image", image)
        cv2.waitKey(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="amiga-camera-stream")
    parser.add_argument("--service-config", type=Path, required=True, help="The camera config.")
    args = parser.parse_args()
    print("started client")
    asyncio.run(main(args.service_config))
