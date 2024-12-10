#!/usr/bin/env python3

# Introduction to Drone Technology IDT
# SDU UAS Center
# University of Southern Denmark
# 2024-11-13 Kjeld Jensen kjen@sdu.dk First version

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import NavSatFix
from utm import UtmConv
import datetime

class GetEveryThingAndListen(Node):
    def __init__(self):
        self.start_time = datetime.datetime.now()
        self.easting = None
        self.norting = None
        self.altitude = None
        self.local_rssi = None
        self.remote_rssi = None

        self.file = open("/home/anders/Documents/Drones_and_Autonomous_Systems/Introduction to Drone Technology/Lecture_08/data.csv", "w")
        self.file.write("Time [s],easting [m],northing [m],altitude [m],local_rssi [dBm],remote_rssi[dBm]\n")


        super().__init__('get_everything_and_log')
        # Subscribe to the /diagnostics topic
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.on_diagnostics,
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.get_logger().info("get_radio_rssi node Started...")
        
        self.sub_global_pos = self.create_subscription(
            NavSatFix,                             # message type
            '/mavros/global_position/global',      # topic to subscribe to
            self.on_global_pos_msg,                # callback function
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        )

    
    def log(self):
        if self.easting !=  None and self.norting != None and self.altitude != None and self.local_rssi != None and self.remote_rssi != None:
            time = datetime.datetime.now() - self.start_time
            self.file.write(str(time) + "," + str(self.easting) + "," + str(self.northing) + "," + str(self.altitude) + "," + str(self.local_rssi) + "," + str(self.remote_rssi) + "\n")
            self.file.flush()
            
            self.time = None
            self.easting = None
            self.norting = None
            self.altitude = None
            self.local_rssi = None
            self.remote_rssi = None


    def on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == "mavros: 3DR Radio":
                for value in status.values:
                    if value.key == "RSSI (dBm)":
                        self.local_rssi = value.value
                        self.get_logger().info(f"Local RSSI (dBm): {self.local_rssi}")
                    elif value.key == "Remote RSSI (dBm)":
                        self.remote_rssi = value.value
                        self.get_logger().info(f"Remote RSSI (dBm): {self.remote_rssi}")
        
        self.log()


    def on_global_pos_msg(self, msg):
        self.altitude = msg.altitude
        (hemisphere, zone, zlet, self.easting, self.northing) = UtmConv.geodetic_to_utm(msg.latitude, msg.longitude)

        self.log()


def main(args=None):
    rclpy.init(args=args)
    rssi_listener = GetEveryThingAndListen()
    rclpy.spin(rssi_listener)

    # Destroy the node explicitly
    rssi_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

