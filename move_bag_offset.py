#!/usr/bin/env python3
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

import argparse
import rclpy
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from builtin_interfaces.msg import Time

def main():
    parser = argparse.ArgumentParser(
        description="Shift /camera timestamps backward by a fixed offset."
    )
    parser.add_argument(
        "--in", dest="input_bag", required=True, help="Path to input bag folder"
    )
    parser.add_argument(
        "--out", dest="output_bag", required=True, help="Path to output bag folder"
    )
    parser.add_argument(
        "--offset", dest="offset", type=float, default=0.6,
        help="Offset in seconds to subtract from /camera header.stamp"
    )
    parser.add_argument(
        "--topic", dest="camera_topic", default="/camera",
        help="Topic to shift (default: /camera)"
    )
    args = parser.parse_args()

    print(f"Input bag: {args.input_bag}")
    print(f"Output bag: {args.output_bag}")
    print(f"Time offset (sec): {args.offset}")
    print(f"Target topic: {args.camera_topic}")

    rclpy.init()

    reader = SequentialReader()
    storage_options = StorageOptions(
        uri=args.input_bag, storage_id="sqlite3"
    )
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)
    topics_types = reader.get_all_topics_and_types()

    writer = SequentialWriter()
    output_options = StorageOptions(
        uri=args.output_bag, storage_id="sqlite3"
    )
    writer.open(output_options, converter_options)
    for topic in topics_types:
        writer.create_topic(topic)

    while reader.has_next():
        topic, data, t = reader.read_next()
        msg_type_str = None
        for meta in topics_types:
            if meta.name == topic:
                msg_type_str = meta.type
                break

        if msg_type_str is None:
            print(f"❌ Could not find message type for topic: {topic}")
            continue

        msg_type = get_message(msg_type_str)

        msg = deserialize_message(data, msg_type)


        msg = deserialize_message(data, msg_type)

        if topic == args.camera_topic:
            # Shift header.stamp backward by offset seconds
            old_sec = msg.header.stamp.sec
            old_nsec = msg.header.stamp.nanosec

            total_nsec = old_sec * 1e9 + old_nsec
            total_nsec += int(args.offset * 1e9)

            new_sec = total_nsec // int(1e9)
            new_nsec = total_nsec % int(1e9)

            msg.header.stamp.sec = int(new_sec)
            msg.header.stamp.nanosec = int(new_nsec)

            print(f"Adjusted {topic} msg: {old_sec}.{old_nsec:09d} -> "
                f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")

        # ✅ Re-serialize it back to CDR bytes:
        new_data = serialize_message(msg)
        writer.write(topic, new_data, t)


    print("✅ Done! New bag written to:", args.output_bag)

if __name__ == "__main__":
    main()
# python shift_camera_stamp.py \
#   --in /home/trailbot/bags/ptp_time \
#   --out /home/trailbot/ptp_modi \
#   --offset 0.6 \
#   --topic /camera
