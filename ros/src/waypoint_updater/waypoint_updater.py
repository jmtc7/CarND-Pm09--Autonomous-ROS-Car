#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import math

# Added dependencies
import numpy as np
from scipy.spatial import KDTree

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribe to the car's pose and the base waypoints
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None


        # Use the 'loop()' function instead of rospy.spin() to control publishing frequency
        self.loop()

    # DONE
    def loop(self):
        # Run at 50Hz while no shutdown order is received
        ## Waypoint_follower runs at ~30Hz, so this freq could be lower
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # If the node is initialized
            if self.pose and self.base_waypoints:
                # Get index of the closest waypooint and publish it
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            
            # Sleep until it is time for the next iteration
            rate.sleep()

    # DONE
    def get_closest_waypoint_idx(self):
        # Get car's position
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        # Search (only) the closest waypoint (1st 1) and get its index (2nd 1)
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        # Check if the closest WP is ahead of the vehicle
        ## Get coordinates of current closest and last one
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        ## Hyperplane through closest and 2nd closest waypoints
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        ## Use dot product to check if the point is behind the car
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        if val > 0: 
            closest_idx = (closest_idx+1) % len(self.waypoints_2d)

        return closest_idx

    # DONE
    def publish_waypoints(self, closest_idx):
        lane = Lane()

        # Use the LOOKAHEAD_WPS WPs to create the lane message
        ## NOTE: Python will provide less than LOOKAHEAD_WPS points if it is required an index out of the list
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx:(LOOKAHEAD_WPS+closest_idx)]

        # Publish
        self.final_waypoints_pub.publish(lane)

    # DONE
    def pose_cb(self, msg):
        self.pose = msg

    # DONE
    def waypoints_cb(self, waypoints):
        print("CREATING WPs TREE")
        # Get base WPs
        self.base_waypoints = waypoints

        # Make sure that the subscriber was initialized by checking if 'waypoints_2d' is created
        if not self.waypoints_2d:
            # Get the 2D coordinates of the base WPs and build a KDTree for quicker searches
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.x] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
