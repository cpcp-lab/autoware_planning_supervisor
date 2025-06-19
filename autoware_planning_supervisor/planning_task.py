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
from tier4_system_msgs.msg import ModeChangeAvailable
from launch.logging import launch_config
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from autoware_planning_supervisor.planning_conf import generate_planning_conf

VL_FILENAME = "vehicle_position_log.csv"
PL_FILENAME = "planning_log.csv"

log_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    #reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=100
)

class VLoggingNode(Node):
    def __init__(self):
        super().__init__('v_logging_node')
        #self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Setting log files.
        self.declare_parameter('supervise_log_dir', launch_config.log_dir)
        ld = self.get_parameter('supervise_log_dir').get_parameter_value().string_value
        self.get_logger().info(f"Preparing: {os.path.join(ld, VL_FILENAME)}")
        self.vlf = open(os.path.join(ld, VL_FILENAME), 'w')

        self.sub_vehicle_log = self.create_subscription(
            Odometry, '/localization/kinematic_state', 
            self.vehicle_log_cb, log_qos)

    def __del__(self):
        self.get_logger().info('Dying...')
        self.vlf.close()
        self.get_logger().info('done')

    def vehicle_log_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        csv.writer(self.vlf).writerow([ts, x, y, yaw])

class PLoggingNode(Node):
    def __init__(self):
        super().__init__('p_logging_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Setting log files.
        self.declare_parameter('supervise_log_dir', launch_config.log_dir)
        ld = self.get_parameter('supervise_log_dir').get_parameter_value().string_value
        self.get_logger().info(f"Preparing: {os.path.join(ld, VL_FILENAME)}")
        self.plf = open(os.path.join(ld, PL_FILENAME), 'w')

        self.sub_planning_log = self.create_subscription(
            Trajectory, '/planning/scenario_planning/trajectory', 
            self.planning_log_cb, log_qos)

    def __del__(self):
        self.get_logger().info('Dying...')
        self.plf.close()
        self.get_logger().info('done')

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

def example_conf():
    init = {
        "position" : {
            "x" : 3778.9994501274487,
            "y" : 73699.79160027765,
            "z" : 19.573500000000003
        },
        "orientation" : {
            "x" : 0.0,
            "y" : 0.0,
            "z" : -0.9712494389103504,
            "w" : 0.2380641245890055
        }
    }
    goal = {
        "position" : {
            "x" : 3733.9041000959696,
            "y" : 73676.27174963453,
            "z" : 19.557499999999997
        },
        "orientation" : {
            "x" : 0.0,
            "y" : 0.0,
            "z" : -0.9712325694336359,
            "w" : 0.2381329378127636
        }
    }
    return (init, goal)

class PlanningTask(Node):
    def __init__(self):
        super().__init__('planning_task_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        #self.sub_map = self.create_subscription(
        #    ModeChangeAvailable, '/system/component_state_monitor/component/launch/map',
        #    self.map_cb, 10)
        #self.map_available = False

        self.pub_init = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_goal = self.create_publisher(
            PoseStamped, '/planning/mission_planning/goal', 10)
        self.pub_engage = self.create_publisher(
            Engage, '/autoware/engage', 10)
        self.initialized = False

        self.pub_done = self.create_publisher(
            Bool, '/planning_task/done', 10)

        #

        self.sub_aw_state = self.create_subscription(
            AutowareState, '/autoware/state', self.state_cb, 10)
        self.prev_state = 0
        self.cb_count = 0

    #def map_cb(self, msg):
    #    self.get_logger().info(f"Map msg: {msg.available}")
    #    if msg.available:
    #        self.map_available = True

    def publish(self, pub, msg):
        while pub.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscriber(s) ...')
            time.sleep(0.5)

        self.get_logger().info(f"Subscriber count: {pub.get_subscription_count()}")

        # Publish.
        self.get_logger().info(f"Publishing: {msg}")
        pub.publish(msg) 
        time.sleep(1)

    def publish_init_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = self.init_pose["position"]["x"]
        msg.pose.pose.position.y = self.init_pose["position"]["y"]
        msg.pose.pose.position.z = self.init_pose["position"]["z"]

        msg.pose.pose.orientation.x = self.init_pose["orientation"]["x"]
        msg.pose.pose.orientation.y = self.init_pose["orientation"]["y"]
        msg.pose.pose.orientation.z = self.init_pose["orientation"]["z"]
        msg.pose.pose.orientation.w = self.init_pose["orientation"]["w"]

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

        msg.pose.position.x = self.goal_pose["position"]["x"]
        msg.pose.position.y = self.goal_pose["position"]["y"]
        msg.pose.position.z = self.goal_pose["position"]["z"]

        msg.pose.orientation.x = self.goal_pose["orientation"]["x"]
        msg.pose.orientation.y = self.goal_pose["orientation"]["y"]
        msg.pose.orientation.z = self.goal_pose["orientation"]["z"]
        msg.pose.orientation.w = self.goal_pose["orientation"]["w"]

        self.publish(self.pub_goal, msg)

    def state_cb(self, msg):
        #self.get_logger().info(f"state: {msg.state}")
        if self.prev_state != AutowareState.DRIVING and msg.state == AutowareState.DRIVING:
            self.cb_count = 0
        else:
            self.cb_count += 1

        #if self.map_available and not self.initialized:
        if not self.initialized:
            self.initialized = True

            #conf = example_conf()
            conf = generate_planning_conf(
                    "/autoware_map/sample-map-planning/lanelet2_map.osm",
                    "/autoware_map/sample-map-planning/map_config.yaml")

            self.get_logger().info(f"conf: {conf}")
            self.init_pose = conf[0]
            self.goal_pose = conf[1]

            # Publish the planning configuration.
            self.publish_init_pose()
            self.publish_goal_pose()

        elif msg.state == AutowareState.WAITING_FOR_ENGAGE:
            # Publish the engage message.
            m = Engage()
            m.engage = True
            self.publish(self.pub_engage, m)

        elif msg.state >= AutowareState.ARRIVED_GOAL or self.cb_count > 1000:
            self.get_logger().info("Shutting down node ...")
            self.get_logger().info(f"cb_count: {self.cb_count}")
            m = Bool()
            m.data = True
            self.publish(self.pub_done, m)

        self.prev_state = msg.state

def main():
    rclpy.init()

    pt_node = PlanningTask()
    v_log_node = VLoggingNode()
    p_log_node = PLoggingNode()

    executor = MultiThreadedExecutor()
    executor.add_node(pt_node)
    executor.add_node(v_log_node)
    executor.add_node(p_log_node)
    try:
        executor.spin()
    finally:
        pt_node.destroy_node()
        v_log_node.destroy_node()
        p_log_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
   main()

# eof
