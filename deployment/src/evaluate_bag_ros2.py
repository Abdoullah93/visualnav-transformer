#!/usr/bin/env python3
import argparse
import os
import csv
from datetime import datetime

import numpy as np

# ROS 2 bag reading
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as e:
    raise RuntimeError("This script must be run in a ROS 2 environment with rosbag2_py available.") from e


def read_bag_metrics(bag_path: str,
                     odom_topic: str = '/odom',
                     cmd_vel_topic: str = '/cmd_vel',
                     reached_topic: str = '/topoplan/reached_goal'):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                                    output_serialization_format='cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Map topic -> type
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    # Resolve message classes
    odom_cls = get_message(topic_types.get(odom_topic, 'nav_msgs/msg/Odometry'))
    cmd_vel_cls = get_message(topic_types.get(cmd_vel_topic, 'geometry_msgs/msg/Twist'))
    reached_cls = get_message(topic_types.get(reached_topic, 'std_msgs/msg/Bool'))

    # Prepare filters if needed
    # reader.set_filter(rosbag2_py.StorageFilter(topics=[odom_topic, cmd_vel_topic, reached_topic]))

    start_time_ns = None
    goal_time_ns = None

    # Path length
    last_pos = None
    path_len = 0.0

    # Control effort (integral of |v|+|w| over time)
    last_cmd = None
    last_cmd_time_ns = None
    ctrl_effort = 0.0

    while reader.has_next():
        (topic, data, t_ns) = reader.read_next()
        if topic not in (odom_topic, cmd_vel_topic, reached_topic):
            continue
        if start_time_ns is None:
            start_time_ns = t_ns

        if topic == odom_topic:
            msg = deserialize_message(data, odom_cls)
            try:
                x = float(msg.pose.pose.position.x)
                y = float(msg.pose.pose.position.y)
                if last_pos is not None:
                    dx = x - last_pos[0]
                    dy = y - last_pos[1]
                    path_len += float(np.hypot(dx, dy))
                last_pos = (x, y)
            except Exception:
                pass

        elif topic == cmd_vel_topic:
            msg = deserialize_message(data, cmd_vel_cls)
            v = float(getattr(msg.linear, 'x', 0.0))
            w = float(getattr(msg.angular, 'z', 0.0))
            if last_cmd_time_ns is not None:
                dt = max(0.0, (t_ns - last_cmd_time_ns) * 1e-9)
                # L1 effort over time
                ctrl_effort += (abs(v) + abs(w)) * dt
            last_cmd = (v, w)
            last_cmd_time_ns = t_ns

        elif topic == reached_topic and goal_time_ns is None:
            msg = deserialize_message(data, reached_cls)
            if bool(getattr(msg, 'data', False)):
                goal_time_ns = t_ns

    success = goal_time_ns is not None
    time_to_goal_s = (goal_time_ns - start_time_ns) * 1e-9 if success else None

    return {
        'success': success,
        'time_to_goal_s': time_to_goal_s,
        'path_length_m': path_len,
        'control_effort_l1': ctrl_effort,
    }


def append_csv(csv_path: str, model_tag: str, bag_path: str, metrics: dict):
    is_new = not os.path.exists(csv_path)
    with open(csv_path, 'a', newline='') as f:
        writer = csv.writer(f)
        if is_new:
            writer.writerow(['timestamp', 'model_tag', 'bag_path', 'success', 'time_to_goal_s', 'path_length_m', 'control_effort_l1'])
        writer.writerow([
            datetime.now().isoformat(timespec='seconds'),
            model_tag,
            os.path.abspath(bag_path),
            int(metrics['success']),
            metrics['time_to_goal_s'] if metrics['time_to_goal_s'] is not None else '',
            metrics['path_length_m'],
            metrics['control_effort_l1'],
        ])


def main():
    parser = argparse.ArgumentParser(description='Evaluate a ROS 2 bag: success, time-to-goal, path length, control effort')
    parser.add_argument('--bag', required=True, help='Path to ros2 bag directory (contains metadata.yaml)')
    parser.add_argument('--model-tag', default='vint', help='Identifier for the run/model')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--reached-topic', default='/topoplan/reached_goal')
    parser.add_argument('--csv-out', default='eval_results.csv', help='Output CSV path')
    args = parser.parse_args()

    metrics = read_bag_metrics(args.bag, args.odom_topic, args.cmd_vel_topic, args.reached_topic)
    print('Evaluation metrics:')
    for k, v in metrics.items():
        print(f'  {k}: {v}')

    append_csv(args.csv_out, args.model_tag, args.bag, metrics)
    print(f"Appended results to {args.csv_out}")


if __name__ == '__main__':
    main()
