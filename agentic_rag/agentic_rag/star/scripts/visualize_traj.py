#!/usr/bin/env python3
import sys
from typing import List, Tuple

import rclpy
from rclpy.time import Time
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

import tf2_ros
import matplotlib.pyplot as plt


def to_time(msg_time) -> Time:
    # msg_time is builtin_interfaces/Time
    return Time(seconds=int(msg_time.sec), nanoseconds=int(msg_time.nanosec))


def quat_to_yaw(qx, qy, qz, qw) -> float:
    # yaw from quaternion (Z axis)
    import math
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    if len(sys.argv) < 2:
        print("Usage: plot_tf_trajectories.py /path/to/rosbag [base_link] [odom] [map]")
        sys.exit(1)

    bag_path = sys.argv[1]
    base_frame = sys.argv[2] if len(sys.argv) > 2 else "base_link"
    odom_frame = sys.argv[3] if len(sys.argv) > 3 else "odom"
    map_frame = sys.argv[4] if len(sys.argv) > 4 else "map"

    rclpy.init()

    # TF buffer to accumulate transforms while we read the bag
    buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=3600.0))

    # Open bag
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id="")
    converter_options = ConverterOptions(input_serialization_format="", output_serialization_format="")
    reader.open(storage_options, converter_options)

    topics_and_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics_and_types}

    if "/tf" not in type_map and "/tf_static" not in type_map:
        print("Bag does not contain /tf or /tf_static.")
        sys.exit(2)

    # Prepare message classes
    TFMessage = get_message("tf2_msgs/msg/TFMessage")

    immediate_xy: List[Tuple[float, float]] = []
    optimized_xy: List[Tuple[float, float]] = []
    corrections_xy: List[Tuple[float, float]] = []  # map->odom translation (optional)

    # Iterate messages in time order
    while reader.has_next():
        topic, data, t_nsec = reader.read_next()

        if topic == "/tf" or topic == "/tf_static":
            msg = deserialize_message(data, TFMessage)

            # Feed transforms into buffer
            for tr in msg.transforms:
                try:
                    if topic == "/tf_static":
                        buffer.set_transform_static(tr, "bag")
                    else:
                        buffer.set_transform(tr, "bag")
                except Exception:
                    # Ignore occasional invalid transforms
                    pass

            # Whenever we see odom->base_link in this batch, try to sample both poses at that stamp
            for tr in msg.transforms:
                if tr.header.frame_id == odom_frame and tr.child_frame_id == base_frame:
                    stamp = to_time(tr.header.stamp)

                    # Immediate: odom->base_link
                    try:
                        T_ob = buffer.lookup_transform(odom_frame, base_frame, stamp)
                        x_o = T_ob.transform.translation.x
                        y_o = T_ob.transform.translation.y
                        immediate_xy.append((x_o, y_o))
                    except Exception:
                        continue

                    # Optimized: map->base_link
                    try:
                        T_mb = buffer.lookup_transform(map_frame, base_frame, stamp)
                        x_m = T_mb.transform.translation.x
                        y_m = T_mb.transform.translation.y
                        optimized_xy.append((x_m, y_m))
                    except Exception:
                        # If map->base_link not available at that time, skip optimized sample
                        pass

                    # Optional: correction map->odom
                    try:
                        T_mo = buffer.lookup_transform(map_frame, odom_frame, stamp)
                        corrections_xy.append((T_mo.transform.translation.x, T_mo.transform.translation.y))
                    except Exception:
                        pass

    rclpy.shutdown()

    if len(immediate_xy) == 0:
        print(f"No samples found for {odom_frame}->{base_frame}. Check frame names.")
        sys.exit(3)

    if len(optimized_xy) == 0:
        print(
            f"Found {len(immediate_xy)} immediate samples but 0 optimized samples.\n"
            f"Likely missing {map_frame}->{odom_frame} or {map_frame}->{base_frame} in the bag."
        )

    # Plot trajectories
    plt.figure()
    ix = [p[0] for p in immediate_xy]
    iy = [p[1] for p in immediate_xy]
    plt.plot(ix, iy, label=f"Immediate ({odom_frame}->{base_frame})")

    if len(optimized_xy) > 0:
        mx = [p[0] for p in optimized_xy]
        my = [p[1] for p in optimized_xy]
        plt.plot(mx, my, label=f"Optimized ({map_frame}->{base_frame})")

    plt.axis("equal")
    plt.legend()
    plt.title("Immediate vs Optimized Trajectory (from /tf)")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.show()

    # Optional: plot correction (map->odom) over time as XY drift
    if len(corrections_xy) > 10:
        plt.figure()
        cx = [p[0] for p in corrections_xy]
        cy = [p[1] for p in corrections_xy]
        plt.plot(cx, cy)
        plt.axis("equal")
        plt.title("Global Correction (map->odom translation)")
        plt.xlabel("dx")
        plt.ylabel("dy")
        plt.show()


if __name__ == "__main__":
    main()
