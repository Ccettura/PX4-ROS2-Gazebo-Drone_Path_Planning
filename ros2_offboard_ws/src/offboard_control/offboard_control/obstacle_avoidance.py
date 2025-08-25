#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint
from px4_msgs.msg import VehicleOdometry, VehicleControlMode
from px4_msgs.msg import VehicleLandDetected  # <-- NEW

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class OffboardNav2Bridge(Node):
    """
    - ARM + OFFBOARD
    - Vertical takeoff and altitude hold at 0.5 m (anchoring XY to current position)
    - ONLY AFTER takeoff: send Nav2 goal and bridge /cmd_vel -> PX4
    - LAND at goal completion + disarm when PX4 detects 'landed'
    """

    PX4_NS = "/chotto/fmu"
    TAKEOFF_ALTITUDE = 0.5          # m AGL (NED target z = -0.5)
    OFFBOARD_RATE_HZ = 30.0         # Hz
    NAV_GOAL = (17.0, 4.0)          # x,y in map
    BASE_LINK_FRAME = "base_link"
    INVERT_YAWSPEED = False         # optional: invert sign if necessary

    # takeoff thresholds
    ALT_EPS = 0.08                  # altitude tolerance in m
    SETTLE_TIME_SEC = 0.8           # settling time at altitude

    def __init__(self):
        super().__init__('offboard_nav2_bridge')

        # TF
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # PX4 publishers
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, f"{self.PX4_NS}/in/offboard_control_mode", 10)
        self.pub_traj = self.create_publisher(
            TrajectorySetpoint, f"{self.PX4_NS}/in/trajectory_setpoint", 10)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, f"{self.PX4_NS}/in/vehicle_command", 10)

        # Nav2 cmd_vel
        qos_cmd = QoSProfile(depth=10)
        qos_cmd.reliability = QoSReliabilityPolicy.RELIABLE
        qos_cmd.history = QoSHistoryPolicy.KEEP_LAST
        self.sub_cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self.cb_cmd_vel, qos_cmd)

        # Nav2 action
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # State
        self.last_cmd_vel_body = Twist()
        self.armed = False
        self.offboard_active = False
        self.goal_sent = False

        self.pos_n = None
        self.pos_e = None
        self.alt_target_ned = -self.TAKEOFF_ALTITUDE
        self.alt_ned = None
        self.takeoff_reached_time = None
        self.takeoff_done = False
        self.vcm_offboard = False

        # NEW: landing/disarming management
        self.landing = False      # are we executing a LAND?
        self.landed = False       # does PX4 report 'landed'?
        self.disarmed = False     # have we already disarmed?

        # continuous streaming
        self.stream_timer = self.create_timer(1.0 / self.OFFBOARD_RATE_HZ, self.stream_offboard)

        # startup sequence
        self.boot_timer = self.create_timer(0.5, self._boot_sequence)

        # PX4 odometry (BEST_EFFORT for QoS compatibility)
        qos_be = QoSProfile(depth=1)
        qos_be.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            VehicleOdometry, f"{self.PX4_NS}/out/vehicle_odometry", self.cb_odom, qos_be)

        # control state (to know if OFFBOARD is really active on PX4 side) - BEST_EFFORT
        qos_vcm = QoSProfile(depth=1)
        qos_vcm.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            VehicleControlMode, f"{self.PX4_NS}/out/vehicle_control_mode", self.cb_vcm, qos_vcm)

        # NEW: land detector (BEST_EFFORT)
        self.create_subscription(
            VehicleLandDetected,
            f"{self.PX4_NS}/out/vehicle_land_detected",
            self.cb_land_detected,
            qos_be)

        self.get_logger().info("OffboardNav2Bridge started.")

    def ts(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    # === Callbacks ===
    def cb_cmd_vel(self, msg: Twist):
        self.last_cmd_vel_body = msg

    def cb_vcm(self, msg: VehicleControlMode):
        was = self.vcm_offboard
        self.vcm_offboard = bool(msg.flag_control_offboard_enabled)
        if self.vcm_offboard and not was:
            self.get_logger().info("OFFBOARD active on PX4 side.")

    def cb_odom(self, msg: VehicleOdometry):
        # NED: x=north, y=east, z=down (+)
        self.pos_n = float(msg.position[0])
        self.pos_e = float(msg.position[1])
        self.alt_ned = float(msg.position[2])

        if not self.takeoff_done:
            # target is negative (−0.3): consider reached when z <= target + eps
            if self.alt_ned is not None and self.alt_ned <= (self.alt_target_ned + self.ALT_EPS):
                if self.takeoff_reached_time is None:
                    self.takeoff_reached_time = self.get_clock().now()
                    self.get_logger().info("Target altitude reached, waiting for settling…")
                else:
                    if (self.get_clock().now() - self.takeoff_reached_time) >= Duration(seconds=self.SETTLE_TIME_SEC):
                        self.takeoff_done = True
                        self.get_logger().info("Takeoff completed. Starting navigation.")
                        if not self.goal_sent:
                            self._send_nav_goal()

    # NEW: land detection callback
    def cb_land_detected(self, msg: VehicleLandDetected):
        was_landed = self.landed
        # some builds have 'landed' as uint8; bool() normalizes it
        self.landed = bool(getattr(msg, 'landed', False))

        if self.landing and self.landed and not was_landed and not self.disarmed:
            self.get_logger().info("Detected 'landed' from PX4: disarming…")
            self._arm(False)
            self.disarmed = True

    # === Boot ===
    def _boot_sequence(self):
        try:
            self.boot_timer.cancel()
        except Exception:
            pass

        self.get_logger().info("Sequence: wait for odometry -> pre-stream -> ARM -> OFFBOARD -> takeoff (hold XY,Z)")

        # Wait for odometry (so we anchor XY to current position)
        self._wait_for_odom(timeout_sec=2.0)

        # 1) pre-stream for OFFBOARD
        self._pre_stream(duration_sec=1.0)

        # 2) ARM
        self._arm(True)

        # 3) OFFBOARD
        self._set_mode_offboard()

        # The Nav2 goal will be sent in cb_odom when takeoff_done becomes True

    def _wait_for_odom(self, timeout_sec: float = 2.0):
        t_end = self.get_clock().now() + Duration(seconds=timeout_sec)
        while self.get_clock().now() < t_end and self.alt_ned is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.alt_ned is None:
            self.get_logger().warn("Odometry not yet available; proceeding anyway.")

    def _pre_stream(self, duration_sec=1.0):
        self.get_logger().info("Pre-stream Offboard setpoints…")
        t_end = self.get_clock().now() + Duration(seconds=duration_sec)
        while self.get_clock().now() < t_end:
            rclpy.spin_once(self, timeout_sec=0.01)
            self.stream_offboard()

    # === PX4 commands ===
    def _send_vehicle_command(self, command: int, param1=0.0, param2=0.0, param3=0.0,
                              param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.ts()
        msg.command = command
        msg.param1 = float(param1); msg.param2 = float(param2); msg.param3 = float(param3)
        msg.param4 = float(param4); msg.param5 = float(param5); msg.param6 = float(param6); msg.param7 = float(param7)
        msg.target_system = 1; msg.target_component = 1
        msg.source_system = 1; msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def _arm(self, arm: bool):
        self._send_vehicle_command(400, 1.0 if arm else 0.0)  # ARM/DISARM
        self.armed = arm
        self.get_logger().info("ARMed" if arm else "DISARMed")

    def _set_mode_offboard(self):
        self._send_vehicle_command(176, 1.0, 6.0, 0.0)  # DO_SET_MODE OFFBOARD
        self.offboard_active = True
        self.get_logger().info("OFFBOARD requested.")

    def _land_and_disarm(self):
        # NO periodic timer: disarm only when 'landed' becomes True
        self.landing = True
        self.get_logger().info("LAND commanded. Waiting for 'landed' state to disarm…")
        self._send_vehicle_command(21, 0.0, 0.0, 0.0)  # NAV_LAND

    # === Nav2 goal ===
    def _send_nav_goal(self):
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 NavigateToPose action server not available.")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(self.NAV_GOAL[0])
        goal.pose.pose.position.y = float(self.NAV_GOAL[1])
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0
        self.get_logger().info(f"Sending Nav2 goal to ({self.NAV_GOAL[0]:.1f}, {self.NAV_GOAL[1]:.1f})…")
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_sent_cb)

    def _goal_sent_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected.")
            return
        self.get_logger().info("Nav2 goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)
        self.goal_sent = True

    def _goal_result_cb(self, future):
        try:
            result_msg = future.result()
            self.get_logger().info(f"Nav2 result: {result_msg.result}")
        except Exception as e:
            self.get_logger().warn(f"No Nav2 result: {e}")
        # On completion: command LAND and disarm ONLY when 'landed'
        self._land_and_disarm()

    # === Streaming setpoint ===
    def stream_offboard(self):
        # During LAND let PX4 handle it; avoid conflicts with AUTO.LAND
        if self.landing:
            return

        now_us = self.ts()

        sp = TrajectorySetpoint()
        sp.timestamp = now_us

        # Important: do not impose fixed yaw -> use NaN; yawspeed will be set in cruise
        sp.yawspeed = 0.0
        sp.yaw = float('nan')

        # TAKEOFF: keep XY fixed at current position, move only Z
        if not self.takeoff_done:
            # use current XY if available, otherwise 0
            x = self.pos_n if self.pos_n is not None else 0.0
            y = self.pos_e if self.pos_e is not None else 0.0
            sp.position = [x, y, self.alt_target_ned]
            sp.velocity = [0.0, 0.0, 0.0]
            sp.yaw = float('nan')
            sp.yawspeed = 0.0

            ocm = OffboardControlMode()
            ocm.timestamp = now_us
            ocm.position = True
            ocm.velocity = False
            ocm.acceleration = False
            ocm.attitude = False
            ocm.body_rate = False
            self.pub_offboard.publish(ocm)
            self.pub_traj.publish(sp)
            return

        # CRUISE: hold Z, use XY velocities from Nav2
        ocm = OffboardControlMode()
        ocm.timestamp = now_us
        ocm.position = True      # keep only Z from position
        ocm.velocity = True      # XY from velocity
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.pub_offboard.publish(ocm)

        # keep target altitude (NED)
        sp.position = [math.nan, math.nan, self.alt_target_ned]

        # cmd_vel in body (base_link, ENU)
        vx_body = self.last_cmd_vel_body.linear.x
        vy_body = self.last_cmd_vel_body.linear.y

        # yaw of base_link in map (ENU)
        try:
            tf = self.tf_buffer.lookup_transform('map', self.BASE_LINK_FRAME, rclpy.time.Time())
            q = tf.transform.rotation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        except (LookupException, ConnectivityException, ExtrapolationException):
            yaw = 0.0

        # body ENU -> map ENU
        vx_map = math.cos(yaw) * vx_body - math.sin(yaw) * vy_body   # East
        vy_map = math.sin(yaw) * vx_body + math.cos(yaw) * vy_body   # North

        # ENU -> NED
        vn = vy_map
        ve = vx_map
        vd = 0.0

        sp.velocity = [vn, ve, vd]

        # yawspeed: ENU (+CCW on Z up) -> NED (+CW on Z down) => invert sign
        yawrate_enu = self.last_cmd_vel_body.angular.z
        yawspeed_ned = -yawrate_enu
        if self.INVERT_YAWSPEED:
            yawspeed_ned = -yawspeed_ned
        sp.yawspeed = yawspeed_ned
        sp.yaw = float('nan')

        self.pub_traj.publish(sp)


def main():
    rclpy.init()
    node = OffboardNav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
