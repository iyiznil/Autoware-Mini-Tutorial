#!/usr/bin/env python3

import numpy as np
import rospy
from threading import Lock

from autoware_mini.msg import Path, VehicleCommand
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

from shapely.geometry import LineString, Point
from shapely import prepare, distance
from scipy.interpolate import interp1d

class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.wheel_base = rospy.get_param("/vehicle/wheel_base")

        # Internal variables
        self.lock = Lock()
        self.path_linestring = None
        self.distance_to_velocity_interpolator = None

        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCommand, queue_size=1)


        # Subscribers
        rospy.Subscriber('path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def path_callback(self, msg):
        if len(msg.waypoints) < 2:
            path_linestring = None
            distance_to_velocity_interpolator = None
        else:
            path_linestring = LineString([(w.position.x, w.position.y) for w in msg.waypoints])
            prepare(path_linestring)


            waypoints_xy = np.array([(w.position.x, w.position.y) for w in msg.waypoints])
            distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints_xy, axis=0)**2, axis=1)))
            distances = np.insert(distances, 0, 0)
            velocities = np.array([w.speed for w in msg.waypoints])
            distance_to_velocity_interpolator = interp1d(distances, velocities, kind='linear', bounds_error=False, fill_value=0.0)

        with self.lock:
            self.path_linestring = path_linestring
            self.distance_to_velocity_interpolator = distance_to_velocity_interpolator

    def current_pose_callback(self, msg):
        if self.path_linestring is None:
            steering_angle = 0.0
            linear_velocity = 0.0
            linear_acceleration = -3.0

            self.distance_to_velocity_interpolator is None

        else:
            current_pose = Point([msg.pose.position.x, msg.pose.position.y])
            d_ego_from_path_start = self.path_linestring.project(current_pose)
            print(d_ego_from_path_start)

            _, _, heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            lookahead_point = self.path_linestring.interpolate(d_ego_from_path_start + self.lookahead_distance)
            lookahead_heading = np.arctan2(lookahead_point.y - msg.pose.position.y, lookahead_point.x - msg.pose.position.x)
            ld = np.sqrt((lookahead_point.x - msg.pose.position.x) ** 2 + (lookahead_point.y - msg.pose.position.y) ** 2)
            steering_angle = np.arctan2(2 * self.wheel_base * np.sin(lookahead_heading - heading) / ld, 1.0)


            linear_velocity = float(self.distance_to_velocity_interpolator(d_ego_from_path_start))
            linear_acceleration = 0.0

            # TODO 5: Use the distance-to-velocity interpolator to get the velocity
            #         at the ego vehicle's position on the path. Replace the constant
            #         linear_velocity with the interpolated value.
            #         Since the interpolator is now used here, add a check for
            #         self.distance_to_velocity_interpolator is None to the if statement above.

        vehicle_cmd = VehicleCommand()

        vehicle_cmd.header.stamp = msg.header.stamp
        vehicle_cmd.header.frame_id = "base_link"
        vehicle_cmd.steering_angle = steering_angle
        vehicle_cmd.speed = linear_velocity
        vehicle_cmd.acceleration = linear_acceleration
        print(vehicle_cmd.steering_angle, msg.pose.position.x, msg.pose.position.y)
        self.vehicle_cmd_pub.publish(vehicle_cmd)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()