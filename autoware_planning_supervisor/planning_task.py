import rclpy
import time
import math, csv
import os
from datetime import datetime
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, UInt8, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from autoware_vehicle_msgs.msg import Engage
from autoware_planning_msgs.msg import Trajectory
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_system_msgs.msg import AutowareState
from launch.logging import launch_config

VL_FILENAME = "vehicle_position_log.csv"
PL_FILENAME = "planning_log.csv"

class LoggingNode(Node):
    def __init__(self):
        super().__init__('my_logging_node')

        # Setting log files.
        self.declare_parameter('supervise_log_dir', launch_config.log_dir)
        ld = self.get_parameter('supervise_log_dir').get_parameter_value().string_value
        self.get_logger().info(f"Preparing: {os.path.join(ld, VL_FILENAME)}")
        self.vlf = open(os.path.join(ld, VL_FILENAME), 'w')
        self.plf = open(os.path.join(ld, PL_FILENAME), 'w')

        #self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.sub_vehicle_log = self.create_subscription(
            Odometry, '/localization/kinematic_state', 
            self.vehicle_log_cb, 10)
        self.sub_planning_log = self.create_subscription(
            Trajectory, '/planning/scenario_planning/trajectory', 
            self.planning_log_cb, 10)


    def __del__(self):
        self.get_logger().info('Dying...')
        self.vlf.close()
        self.plf.close()
        self.get_logger().info('done')

    def vehicle_log_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        csv.writer(self.vlf).writerow([ts, x, y, yaw])

    def planning_log_cb(self, msg):
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        writer = csv.writer(self.plf)
        for pt in msg.points:
            x = pt.pose.position.x
            y = pt.pose.position.y
            ori = pt.pose.orientation
            yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
            v = pt.longitudinal_velocity_mps
            writer.writerow([ts, x, y, yaw, v])

#

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp) * 180 / math.pi

class PlanningTask(Node):
    def __init__(self):
        super().__init__('planning_task_node')

        self.pub_init = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_goal = self.create_publisher(
            PoseStamped, '/planning/mission_planning/goal', 10)
        self.pub_engage = self.create_publisher(
            Engage, '/autoware/engage', 10)

        self.pub_done = self.create_publisher(
            Bool, '/planning_task/done', 10)

        # Publish the planning configuration.
        self.publish_init_pose()
        self.publish_goal_pose()

        #

        #self.sub_state = self.create_subscription(
        #    OperationModeState, '/api/operation_mode/state', self.op_mode_cb, 10)
        #self.current = None
        self.sub_a_state = self.create_subscription(
            AutowareState, '/autoware/state', self.state_cb, 10)

        #time.sleep(3)
        ## Publish the engage msg.
        #msg = Engage()
        #msg.engage = True
        #self.publish(self.pub_engage, msg)

    #def __del__(self):
    #    self.get_logger().info('Dying...')
    #    self.vlf.close()
    #    self.plf.close()
    #    self.get_logger().info('done')

    def publish(self, pub, msg):
        while pub.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscriber(s) ...')
            time.sleep(0.5)

        # Publish.
        self.get_logger().info(f"publishing: {msg}")
        pub.publish(msg) 
        time.sleep(0.1)

    def publish_init_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = 3778.9994501274487
        msg.pose.pose.position.y = 73699.79160027765
        msg.pose.pose.position.z = 19.573500000000003

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = -0.9712494389103504
        msg.pose.pose.orientation.w = 0.2380641245890055

        msg.pose.covariance = [
            0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]

        self.publish(self.pub_init, msg)

    def publish_goal_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        #msg.pose.position.x = 3823.2120500461315
        #msg.pose.position.y = 73783.51585000893
        #msg.pose.position.z = 19.2895

        #msg.pose.orientation.x = 0.0
        #msg.pose.orientation.y = 0.0
        #msg.pose.orientation.z = 0.8542348084262681
        #msg.pose.orientation.w = 0.519887384029404

        msg.pose.position.x = 3733.9041000959696
        msg.pose.position.y = 73676.27174963453
        msg.pose.position.z = 19.557499999999997

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = -0.9712325694336359
        msg.pose.orientation.w = 0.2381329378127636

        self.publish(self.pub_goal, msg)

    def vehicle_log_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        csv.writer(self.vlf).writerow([ts, x, y, yaw])

    def planning_log_cb(self, msg):
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        writer = csv.writer(self.plf)
        for pt in msg.points:
            x = pt.pose.position.x
            y = pt.pose.position.y
            ori = pt.pose.orientation
            yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
            v = pt.longitudinal_velocity_mps
            writer.writerow([ts, x, y, yaw, v])

    #def op_mode_cb(self, msg):
    #    prev = self.current
    #    self.current = msg.mode
    #    self.get_logger().info(f"op mode: {msg.mode}")
    #    if prev is not None and prev > 1 and self.current == 1:
    #        self.get_logger().info("Shutting down node ...")
    #        msg = Bool()
    #        msg.data = True
    #        self.publish(self.pub_done, msg)

    def state_cb(self, msg):
        #self.get_logger().info(f"state: {msg.state}")
        if msg.state == AutowareState.WAITING_FOR_ENGAGE:
            # Publish the engage msg.
            msg = Engage()
            msg.engage = True
            self.publish(self.pub_engage, msg)
        elif msg.state >= AutowareState.ARRIVED_GOAL:
            self.get_logger().info("Shutting down node ...")
            msg = Bool()
            msg.data = True
            self.publish(self.pub_done, msg)

def main():
    rclpy.init()

    #node = PlanningTask()
    #rclpy.spin(node)
    #node.destroy_node()
    #rclpy.shutdown()

    pt_node = PlanningTask()
    log_node = LoggingNode()

    executor = MultiThreadedExecutor()
    executor.add_node(pt_node)
    executor.add_node(log_node)
    try:
        executor.spin()
    finally:
        pt_node.destroy_node()
        log_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
   main()

# eof
