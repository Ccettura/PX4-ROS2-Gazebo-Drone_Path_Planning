#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class ProjectedToMap(Node):
    def __init__(self):
        super().__init__('projected_to_map')
        # Sub: qualsiasi QoS va bene (publisher è Reliable/Volatile)
        sub_qos = QoSProfile(depth=10)
        # Pub: latched per i late-joiner (Nav2)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map', pub_qos)
        self.sub = self.create_subscription(OccupancyGrid, '/projected_map', self.cb, sub_qos)

    def cb(self, msg: OccupancyGrid):
        # Se vuoi forzare il frame a 'map', lascialo così; altrimenti rimuovi la riga sotto
        msg.header.frame_id = 'map'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ProjectedToMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
