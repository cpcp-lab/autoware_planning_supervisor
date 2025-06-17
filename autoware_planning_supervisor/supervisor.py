import os
import asyncio
import threading
import time
import rclpy
import argparse
import datetime
from launch import LaunchService
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool

# Lister for the done topic for planning tasks.
class ListenerNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')
        self.subscription = self.create_subscription(
            Bool, '/planning_task/done', self.callback, 10)

    def callback(self, msg):
        global launch_service
        self.get_logger().info(f'Listener received: {msg.data}')
        launch_service.shutdown()

def start_rclpy_listener():
    #time.sleep(1)

    rclpy.init()
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#

launch_file_path = os.path.join(
    get_package_share_directory('autoware_planning_supervisor'),
    '',
    'planning_simulator.launch.xml'
)
#launch_file_path = os.path.join(
#    os.getenv("HOME"), "ros2_ws", "install", "my_package", "share", "my_package", "launch", "my_launch_file.launch.xml"
#)

launch_service = None

# Execute the planning task by adding to LaunchService.
async def launch(log_dir):
    global launch_service
    launch_service = LaunchService()

    await asyncio.sleep(1)

    # Read a LaunchDescription.
    included_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(launch_file_path),
        launch_arguments={
            'map_path': '/autoware_map/sample-map-planning',
            'vehicle_model': 'sample_vehicle',
            'sensor_model': 'sample_sensor_kit',
            'supervise_log_dir': log_dir
            }.items()
    )
    launch_service.include_launch_description(included_launch)

    task = asyncio.create_task(launch_service.run_async())
    await task
    print("A launch service finished.")
    await asyncio.sleep(3)

#

def main():
    # Setting the argument parser.
    parser = argparse.ArgumentParser()
    parser.add_argument('--log_dir', type=str, help='Specify the log dir')
    args = parser.parse_args()

    log_base = args.log_dir
    if not log_base or not os.path.isdir(log_base):
        log_base = os.get_env("ROS_LOG_DIR")
    
    now = datetime.datetime.now()
    datetime_str = now.strftime('%Y-%m-%d-%H-%M-%S')

    # Invoke the listener node.
    t = threading.Thread(target=start_rclpy_listener)
    t.start()

    try:
        i = 0
        while True:
            # Create a subdir for logging.
            while True:
                i += 1
                dirname = f"supervise-{datetime_str}-{i}"
                log_dir = os.path.join(log_base, dirname)
                if not os.path.exists(log_dir):
                    os.mkdir(log_dir)
                    break

            asyncio.run( launch(log_dir) )

    except KeyboardInterrupt:
        print("Interrupted. Finishing the process.")

if __name__ == '__main__':
    main()

# eof
