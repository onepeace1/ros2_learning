
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
import csv
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, Vector3, Quaternion
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleCommand
import time
import math

class MissionOne(Node):
    def __init__(self):
        super().__init__('mission_one')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/px4_visualizer/vehicle_pose',
            self.position_callback,
            10
        ) #should be fixed to local position

        # Publishers
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/offboard_velocity_cmd',
            qos_profile
        )

        self.position_publisher = self.create_publisher(
            PoseStamped,
            '/offboard_position_cmd',
            qos_profile
        )

        self.arm_publisher = self.create_publisher(
            Bool,
            '/arm_message',
            qos_profile
        )

        self.vtol_publisher_fw = self.create_publisher(
            Bool,
            '/vtol_message_fw',
            qos_profile
        )

        self.vtol_publisher_mc = self.create_publisher(
            Bool,
            '/vtol_message_mc',
            qos_profile
        )


        # MAIN LOGIC 

        

        self.csv_file_path = self.get_csv_file_path()
        self.csv_data = self.csv_to_mem(self.csv_file_path)

        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'z': 5.0}, #WP1
            {'x':-2.00, 'y': 0.0, 'z': 5.0}, #WP2
            {'x': -2.00, 'y': 2.00, 'z': 5.0}, #WP3
            {'x': 2.00, 'y': 2.00, 'z': 5.0}, #WP4
            {'x': 2.00, 'y': -2.00, 'z': 5.0}, #WP5
            {'x': -2.00, 'y': -2.00, 'z': 5.0}, #WP6
            {'x': -2.00, 'y': 2.00, 'z': 5.0} #WP7

        ]

        self.curr_way_index = 0 #change
        self.position_tolerance = 0.1
        self.s_position_tolerance = 35.0
        self.vtol_count = 0
        self.waypoint_ind = 0
        self.max_waypoint = self.s_way_len(self.waypoints)
        self.curr_height = 0.0
        self.actual_height = 0.0
        self.m_target = self.waypoints[0]
        self.timerCount = 0

        time.sleep(5)
        self.get_logger().info("Main launched")
        self.arm_drone(True)
        time.sleep(10)
        self.navigate_waypoints()

    def position_callback(self, msg):
        self.current_position['x'] = msg.pose.position.x
        self.current_position['y'] = msg.pose.position.y
        self.current_position['z'] = msg.pose.position.z
        self.current_orientation = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
    
    def get_csv_file_path(self):
        package_share_directory = get_package_share_directory('mission_control')
        csv_file_path = os.path.join(package_share_directory, 'test_clothoid_result.csv')
        return csv_file_path

    def csv_to_mem(self, csv_file_path):
        with open(csv_file_path, mode='r') as file:
            csv_reader = list(csv.reader(file))
        return csv_reader
    
    def way_data(self, data, col, curr_height):
        if col < len(data):
            row = data[col]
            if len(row) >= 2:
                x = round(float(row[0]), 2)
                y = round(float(row[1]), 2)
                z = curr_height
                return {'x': x, 'y': y, 'z': z}
        return None
    
    def s_way_len(self, data):
        return len(data)


    def arm_drone(self, arm):
        arm_msg = Bool()
        arm_msg.data = arm
        self.arm_publisher.publish(arm_msg)
        self.get_logger().info("Drone armed")

    def disarm_drone(self, arm):
        arm_msg = Bool()
        arm_msg.data = arm
        self.arm_publisher.publish(arm_msg)
        self.get_logger().info("Drone disarmed")

    def arm_vtol(self, varm):
        vtol_hmg = Bool()
        vtol_hmg.data = varm
        self.vtol_publisher_fw.publish(vtol_hmg)
        self.get_logger().info("Go VTOL")
    
    def arm_mc(self, marm):
        vtol_mmg = Bool()
        vtol_mmg.data = marm
        self.vtol_publisher_mc.publish(vtol_mmg)
        self.get_logger().info("Go MC")

    def navigate_waypoints(self):
        self.wp_timer = self.create_timer(0.01, self.navigate_waypoint_callback)


    def navigate_waypoint_callback(self):
        
        if self.curr_way_index < self.max_waypoint:
            # self.curr_height = self.m_target['z']
            # if self.actual_height != self.curr_height:
            #     adjustment = 0.05 if self.actual_height < self.curr_height else -0.05
            #     self.actual_height = round(self.actual_height + adjustment, 2)

            self.m_target = self.waypoints[self.curr_way_index]

            if self.curr_way_index == 0:
                if self.is_waypoint_reached(self.m_target):
                    self.curr_way_index += 1
                    
                    self.get_logger().info(f"x->{self.current_position['x']},y->{self.current_position['y']},z->{self.current_position['z']}")
                    self.get_logger().info("Takeoff Completed")
            
                else:
                    pose = self.calculate_position_command_f(self.m_target)
                    self.position_publisher.publish(pose)

            if self.curr_way_index != 0:
                if self.is_waypoint_reached(self.m_target):
                    self.curr_way_index += 1
                    
                    self.get_logger().info(f"x->{self.current_position['x']},y->{self.current_position['y']},z->{self.current_position['z']}")
                    self.get_logger().info(f"Waypoint {self.curr_way_index} reached")
                else: 
                    pose = self.calculate_position_command(self.m_target)
                    self.position_publisher.publish(pose)
        else:
            self.get_logger().info("All waypoints reached")
            self.destroy_timer(self.wp_timer)

    def calculate_velocity_command(self, target):
        twist = Twist()
        kp = 0.2
        error_y = target['y'] - self.current_position['y']
        error_z = target['z'] - self.current_position['z']
        error_x = target['x'] - self.current_position['x']

        twist.linear.x = kp * error_x
        twist.linear.y = kp * -error_y
        twist.linear.z = kp * error_z

        max_speed = 1.0
        norm = math.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        if norm > max_speed:
            twist.linear.x = (twist.linear.x / norm) * max_speed
            twist.linear.y = (twist.linear.y / norm) * max_speed
            twist.linear.z = (twist.linear.z / norm) * max_speed

        twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        return twist
    
    def calculate_position_command(self, target):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target['y']
        pose.pose.position.y = target['x']
        pose.pose.position.z = target['z']

        # Calculate desired yaw
        error_y = target['y'] - self.current_position['y']
        error_x = target['x'] - self.current_position['x']
        desired_yaw = math.atan2(error_y, error_x)

        # Set yaw in Euler angles directly
        pose.pose.orientation.x = desired_yaw + 1.5708
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        return pose
    
    def calculate_position_command_f(self, target):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target['y'] 
        pose.pose.position.y = target['x']
        pose.pose.position.z = target['z'] 

        pose.pose.orientation.x = 1.5708

        return pose

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        roll, pitch, yaw: in radians
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        
        quaternion = Quaternion()
        quaternion.x = qx
        quaternion.y = qy
        quaternion.z = qz
        quaternion.w = qw

        return quaternion

    def is_waypoint_reached(self, target):
        enu_current_position_x = -self.current_position['y']  
        enu_current_position_y = self.current_position['x']  
        enu_current_position_z = self.current_position['z'] 
        x_dist = abs(target['x'] - enu_current_position_x)
        y_dist = abs(target['y'] - enu_current_position_y)
        z_dist = abs(target['z'] - enu_current_position_z)

        return (x_dist < self.position_tolerance and
                y_dist < self.position_tolerance and
                z_dist < self.position_tolerance)

    def s_is_waypoint_reached(self, target):

        enu_current_position_x = -self.current_position['y']  
        enu_current_position_y = self.current_position['x']  

        x_dist = abs(target['x'] - enu_current_position_x)
        y_dist = abs(target['y'] - enu_current_position_y)

        return (x_dist < self.s_position_tolerance and
                y_dist < self.s_position_tolerance)

    


def main(args=None):
    rclpy.init(args=args)
    mission1node = MissionOne()
    rclpy.spin(mission1node)
    print("node destroyed")
    mission1node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()