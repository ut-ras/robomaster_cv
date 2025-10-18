import rosbag2_py
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
from pathlib import Path
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

def get_reader(bag_path):
    """
    Determines if the bag file is compressed or not and returns the appropriate reader.
    """
    bag_folder = Path(bag_path)
    db3_file = next(bag_folder.glob("*.db3"), None)
    zstd_file = next(bag_folder.glob("*.db3.zstd"), None)

    if zstd_file:
        logging.info("Using SequentialCompressionReader for compressed .db3.zstd file.")
        reader = rosbag2_py.SequentialCompressionReader()
    elif db3_file:
        logging.info("Using SequentialReader for uncompressed .db3 file.")
        reader = rosbag2_py.SequentialReader()
    else:
        logging.error("No compatible .db3 or .db3.zstd files found in the directory.")
        return None

    return reader

def read_images_from_topic(bag_path, topic_name):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = get_reader(bag_path)
    if reader is None:
        return []

    reader.open(storage_options, converter_options)

    bridge = CvBridge()
    images = []

    # Get and verify topic type
    topic_types = reader.get_all_topics_and_types()
    topic_type = None
    for t in topic_types:
        if t.name == topic_name:
            topic_type = t.type
            break
    if not topic_type:
        logging.error(f"Topic '{topic_name}' not found.")
        return []

    msg_type = get_message(topic_type)
    logging.info(f"Deserializing messages from topic '{topic_name}'")

    message_count = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == topic_name:
            message = deserialize_message(data, msg_type)

            try:
                if isinstance(message, Image):
                    cv_image = bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                elif isinstance(message, CompressedImage):
                    np_arr = np.frombuffer(message.data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                else:
                    logging.warning(f"Unsupported message type at timestamp {timestamp}.")
                    continue

                images.append((timestamp, cv_image))
                message_count += 1

                if message_count % 50 == 0:
                    logging.info(f"Processed {message_count} images so far...")

            except Exception as e:
                logging.error(f"Failed to convert message to image at timestamp {timestamp}: {e}")
                continue

    logging.info(f"Total images retrieved: {len(images)}")
    return images

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--topic", type=str, help="Image/CompressedImage topic name")
    parser.add_argument("-o", "--out_file", type=str, help="output mp4 file name", default="output.mp4")
    parser.add_argument("-f", "--fps", type=int, help="output file fps", default=30)
    parser.add_argument("bag_file", type=str, help="input bag directory")
    return parser.parse_args()

def write_video(images, out_file, fps):
    if not images:
        logging.warning("No images to write to video.")
        return

    frame_height, frame_width = images[0][1].shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'avc1')
    out = cv2.VideoWriter(out_file, fourcc, fps, (frame_width, frame_height))

    logging.info(f"Writing video to {out_file} at {fps} FPS")
    for i, (timestamp, img) in enumerate(images):
        out.write(img)

        if (i + 1) % 50 == 0:
            logging.info(f"Wrote {i + 1} frames to video...")

    out.release()
    logging.info(f"Video successfully written to {out_file}")

if __name__ == "__main__":
    args = get_args()

    bag_file = args.bag_file
    out_file = args.out_file
    fps = args.fps

    logging.info("Starting to process the rosbag")
    image_messages = read_images_from_topic(str(bag_file), args.topic)
    write_video(image_messages, out_file, fps)
    logging.info("Process complete")
