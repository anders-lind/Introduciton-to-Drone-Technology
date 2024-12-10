import math
import rclpy
import csv
import datetime

# from utm import UtmConv
from .utm import UtmConv

from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3, Point
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix


class ros_node_class(Node):
    def __init__(self):
        super().__init__('idt')

        print("init 1")

        self.uc = UtmConv()
        self.msg_system_state = State()
        self.msg_fcu_orientation = Vector3()
        self.time_started = datetime.datetime.now()
        self.position = [0, 0, 0]
        self.position_raw = [0, 0, 0]
        self.position_first = [0, 0, 0]
        self.position_raw_first = [0, 0, 0]
        self.position_initialized = False
        self.position_raw_initialized = False


        self.file = open("/home/anders/Documents/Drones_and_Autonomous_Systems/IDT/Lecture_07/data.csv", "w")
        self.file.write("Time [s], roll [deg], pitch [deg], yaw [deg], pos_x [m], pos_y [m], pos_z [m], pos_raw_x [m], pos_raw_y [m], pos_raw_z [m], position_initialized [b], position_raw_initialized [b], status \n")

        # publishers

        # subscribers

        # system state
        self.sub_system_state = self.create_subscription(
            State,
            '/mavros/state',
            self.on_system_state_msg,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # local position pose
        self.sub_local_pos_pose = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.on_local_pos_pose_msg,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # subscribers
        self.sub_global_pos = self.create_subscription(
            NavSatFix,                             # message type
            '/mavros/global_position/global',      # topic to subscribe to
            self.on_global_pos_msg,                # callback function
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.sub_global_pos_raw = self.create_subscription(
            NavSatFix,                              # message type
            '/mavros/global_position/raw/fix',      # topic to subscribe to
            self.on_global_pos_raw_msg,             # callback function 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.get_logger().warn('Hello Drone!')

        # timers
        self.timer = self.create_timer(0.1, self.timer_update)

        print("init 2")


    def on_system_state_msg(self, msg):
        self.msg_system_state = msg


    def on_local_pos_pose_msg(self, msg):
        # convert quaternion to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)

        # store for use elsewhere
        self.msg_fcu_orientation.x = math.degrees(roll)
        self.msg_fcu_orientation.y = math.degrees(pitch)
        self.msg_fcu_orientation.z = math.degrees(yaw)
    

    def on_global_pos_msg(self, msg):
        (hemisphere, zone, letter, east, north) = self.uc.geodetic_to_utm (msg.latitude, msg.longitude)
        
        if (not self.position_initialized and (east == 0 or north == 0)):
            self.position_first = [east, north, msg.altitude]
            self.position_initialized = True

        self.position = [east-self.position_first[0], north-self.position_first[1], msg.altitude-self.position_first[2]]


    def on_global_pos_raw_msg(self, msg):
        (hemisphere, zone, letter, east, north) = self.uc.geodetic_to_utm (msg.latitude, msg.longitude)

        if (not self.position_raw_initialized and (east != 0 or north != 0)):
            self.position_raw_first = [east, north, msg.altitude]
            self.position_raw_initialized = True

        self.position_raw = [east-self.position_raw_first[0], north-self.position_raw_first[1], msg.altitude-self.position_raw_first[2]]


    def quaternion_to_euler(self, q_x, q_y, q_z, q_w):
        """Convert quaternion to euler angles""" 
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q_w * q_x + q_y * q_z)
        cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q_w * q_y - q_z * q_x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw


    def timer_update(self):
        """
        Callback function for the timer. 
        Prints the FCU orientation and system state every second.""" 

        print("update: " + str(self.position_initialized) + ", " + str(self.position_raw_initialized))

        # Runs every second
        print("System state: ", self.msg_system_state.mode)
        print(  f"roll: {self.msg_fcu_orientation.x:.2f}, "
                f"pitch: {self.msg_fcu_orientation.y:.2f}, "
                f"yaw: {self.msg_fcu_orientation.z:.2f}"    )
        print("pos:     ", self.position)
        print("pos_raw: ", self.position_raw)

        time = datetime.datetime.now() - self.time_started
        self.file.write(str(time.total_seconds()) + "," + str(self.msg_fcu_orientation.x) + "," + str(self.msg_fcu_orientation.y) + "," + str(self.msg_fcu_orientation.z) + "," + str(self.position[0]) + "," + str(self.position[1]) + "," + str(self.position[2]) + "," + str(self.position_raw[0]) + "," + str(self.position_raw[1]) + "," + str(self.position_raw[2]) + "," + str(self.position_initialized) + "," + str(self.position_raw_initialized) + "," + str(self.msg_system_state.mode) + "\n")
        self.file.flush()



def main(args=None):
    rclpy.init(args=args)
    ros_node = ros_node_class()
    rclpy.spin(ros_node)

    ros_node.file.close()

    ros_node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()
