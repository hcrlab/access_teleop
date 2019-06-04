#! /usr/bin/env python

import actionlib
import copy
import geometry_msgs.msg
import math
import nav_msgs.msg
import numpy as np
import rospy
import tf.transformations as tft

from copy import deepcopy

ANGLELIMIT = 0.1  # maximum error allowed in the robot's current yaw and the desired yaw (in radian)

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()

        base.go_forward(0.1)
        base.turn(30 * math.pi / 180)

        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        self._publisher = rospy.Publisher(
            'cmd_vel', geometry_msgs.msg.Twist, queue_size=5)
        self.odom_sub = rospy.Subscriber(
            'odom',
            nav_msgs.msg.Odometry,
            callback=self._odom_callback,
            queue_size=10)
        self.odom = None

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self._publisher.publish(twist)

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance.

        You cannot use this method to move less than 1 cm.

        Args:
            distance: The distance, in meters, to rotate. A positive value
                means forward, negative means backward.
            max_speed: The maximum speed to travel, in meters/second.
        """
        while self.odom is None:
            rospy.sleep(0.1)
        start = copy.deepcopy(self.odom)
        rate = rospy.Rate(25)
        distance_from_start = self._linear_distance(start, self.odom)
        while distance_from_start < math.fabs(distance):
            distance_from_start = self._linear_distance(start, self.odom)
            if distance_from_start >= math.fabs(distance):
                return
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()

    # def turn(self, angular_distance, speed=1):
    #     """Rotates the robot a certain angle.

    #     This illustrates how to turn the robot by checking that the X-axis of
    #     the robot matches that of the goal.

    #     Args:
    #         angular_distance: The angle, in radians, to rotate. A positive
    #             value rotates counter-clockwise.
    #         speed: The maximum angular speed to rotate, in radians/second.
    #     """
    #     while self.odom is None:
    #         rospy.sleep(0.1)
    #     direction = -1 if angular_distance < 0 else 1

    #     current_yaw = self._yaw_from_quaternion(self.odom.orientation)
    #     goal_yaw = current_yaw + angular_distance
    #     goal_x_axis = np.array([math.cos(goal_yaw), math.sin(goal_yaw), 0])

    #     rate = rospy.Rate(100)
    #     x_axis = self._x_axis_from_quaternion(self.odom.orientation)
    #     while not np.allclose(x_axis, goal_x_axis, atol=0.01):
    #         self.move(0, direction * speed)
    #         x_axis = self._x_axis_from_quaternion(self.odom.orientation)
    #         rate.sleep()

    # ----- A second version of turn() -----
    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        while self.odom is None:
            rospy.sleep(0.1)
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.odom)
        direction = -1 if angular_distance < 0 else 1
        mat = tft.quaternion_matrix([self.odom.orientation.x, self.odom.orientation.y, self.odom.orientation.z, self.odom.orientation.w])
        angleCur = math.atan2(mat[1, 0], mat[0, 0])
        angleEnd = (angleCur + angular_distance)

        if math.fabs(angleEnd) > math.pi:
            angleEnd -= direction*(2*math.pi)

        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        while not self.reachGoal(angleCur, angleEnd, direction):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            self.move(0, direction * speed)
            rate.sleep()    
            mat = tft.quaternion_matrix([self.odom.orientation.x, self.odom.orientation.y, self.odom.orientation.z, self.odom.orientation.w])
            angleCur = math.atan2(mat[1, 0], mat[0, 0])

    def reachGoal(self, angleCur, angleEnd, direction):
        if angleCur*angleEnd < 0:
            return False
        if direction > 0:
            angleCur += ANGLELIMIT;
            if angleEnd > 0:
                if angleCur > angleEnd:
                    return True
            else:
                if angleCur > angleEnd:
                    return True
        else:
            angleCur -= ANGLELIMIT;
            if angleEnd > 0:
                if angleCur < angleEnd:
                    return True
            else:
                if angleCur < angleEnd:
                    return True

        return False
    # (end)

    def turn_alternate(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        This illustrates how to turn the robot using yaw angles and careful
        accounting.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The maximum angular speed to rotate, in radians/second.
        """
        while self.odom is None:
            rospy.sleep(0.1)
        direction = -1 if angular_distance < 0 else 1

        current_coord = self._yaw_from_quaternion(self.odom.orientation) % (
            2 * math.pi)
        end_coord = (current_coord + angular_distance) % (2 * math.pi)
        rate = rospy.Rate(25)

        while True:
            current_coord = self._yaw_from_quaternion(
                self.odom.orientation) % (2 * math.pi)
            remaining = (direction *
                         (end_coord - current_coord)) % (2 * math.pi)
            if remaining < 0.01:
                return
            speed = max(0.25, min(1, remaining))
            self.move(0, direction * speed)
            rate.sleep()

    def turn_p_controller(self, angular_distance, speed=0.5):
        """P controller for rotating the robot a certain angle."""
        while self.odom is None:
            rospy.sleep(0.1)

        kP_theta = 1.5

        curr_orientation = self.odom.orientation
        curr_angle = self._yaw_from_quaternion(curr_orientation)
        error = angular_distance - curr_angle

        while math.fabs(error) > 0.05:
            self.move(0, kP_theta * error)
            rospy.sleep(0.03)

            curr_orientation = self.odom.orientation
            curr_angle = self._yaw_from_quaternion(curr_orientation)
            error = angular_distance - curr_angle

        self.stop()

    def align_with_x_axis_pos(self):
        """Rotates the robot so that it aligns with positive X-axis."""
        self.align_with_axis(0, 1)

    def align_with_y_axis_pos(self):
        """Rotates the robot so that it aligns with positive Y-axis."""
        self.align_with_axis(1, 1)

    def align_with_y_axis_neg(self):
        """Rotates the robot so that it aligns with negative Y-axis."""
        self.align_with_axis(1, -1)

    def align_with_axis(self, axis, dir, speed=0.5):
        """Rotates the robot so that it aligns with X or Y-axis (as specified).

        Args:
            speed: The maximum angular speed to rotate, in radians/second.
        """
        while self.odom is None:
            rospy.sleep(0.1)

        curr_orientation = self.odom.orientation
        curr_angle = self._yaw_from_quaternion(curr_orientation)

        desire_orientation = deepcopy(self.odom.orientation)
        desire_orientation.x = 0
        desire_orientation.y = 0
        desire_orientation.z = 0
        desire_orientation.w = 1
        desire_angle = self._yaw_from_quaternion(desire_orientation)

        rotate_angle = (desire_angle - curr_angle) % (2 * math.pi)
        if axis is 1:  # y-axis
            rotate_angle = rotate_angle + dir * math.pi / 2

        # self.turn_p_controller(rotate_angle)

        # adjust the robot if the angle difference is greater than 10 degree (or pi/18 radian)
        if rotate_angle > 0.1:
            self.turn(rotate_angle, speed)

        # self.stop()

    def stop(self):
        """Stops the mobile base from moving.
        """
        self.move(0, 0)

    def _odom_callback(self, msg):
        self.odom = msg.pose.pose

    @staticmethod
    def _linear_distance(pose1, pose2):
        pos1 = pose1.position
        pos2 = pose2.position
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _x_axis_from_quaternion(q):
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        return m[:3, 0]  # First column

    @staticmethod
    def _yaw_from_quaternion(q):
        m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        return math.atan2(m[1, 0], m[0, 0])

    @staticmethod
    def _angular_distance(pose1, pose2):
        q1 = pose1.orientation
        q2 = pose2.orientation
        y1 = Base._yaw_from_quaternion(q1)
        y2 = Base._yaw_from_quaternion(q2)
        return math.fabs(y1 - y2) % (2 * math.pi)
