#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Hmmm, don't really know the msg type I guess.
        #rospy.Subscriber('/traffic_waypoint', int32, self.traffic_cb)

        # There is no topic (yet) for /obstacle_waypoint
        #rospy.Subscriber('/obstacle_waypoint', ???, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = []
        self.x_ave = 0.0
        self.y_ave = 0.0
        self.phi = []
        self.wrap = 0
        rospy.spin()

    # Linear search... we should do better!
    def get_index(self, phi):
        idx = 0 if phi <= self.phi[0] else self.wrap
        while phi < self.phi[idx]:
            idx += 1
        return idx
        
    def pose_cb(self, msg):
        # pose_cb might be called before waypoints_cb
        if self.waypoints == []:
            return None
        x = msg.pose.position.x - self.x_ave
        y = msg.pose.position.y - self.y_ave
        phi = math.atan2(x, y)
        idx = self.get_index(phi)
        self.publish(idx)
        
    def waypoints_cb(self, lane):
        wp = lane.waypoints
        x_tot = 0.0
        y_tot = 0.0
        for p in wp:
            x_tot += p.pose.pose.position.x
            y_tot += p.pose.pose.position.y

        self.x_ave = x_tot / len(wp)
        self.y_ave = y_tot / len(wp)

        for p in wp:
            x = p.pose.pose.position.x - self.x_ave
            y = p.pose.pose.position.y - self.y_ave
            phi = math.atan2(x, y)
            self.phi.append(phi)
            if (len(self.phi) > 1) and self.phi[-1] * self.phi[-2] < 0.0:
                self.wrap = len(self.phi) - 1
                
        self.waypoints.extend(wp)
        # make wrap around easier by extending waypoints
        self.waypoints.extend(wp[0:LOOKAHEAD_WPS])

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

    def publish(self, idx):
        lane = Lane()
        lane.header.frame_id = '/stuff'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = []
        lane.waypoints.extend(self.waypoints[idx:idx+LOOKAHEAD_WPS])
        for i in range(LOOKAHEAD_WPS):
            self.set_waypoint_velocity(lane.waypoints, i, 30.0)

        self.final_waypoints_pub.publish(lane)
        

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
