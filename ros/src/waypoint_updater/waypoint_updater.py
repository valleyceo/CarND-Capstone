#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from copy import deepcopy

import math

'''This node will publish waypoints from the car's current position
to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version
which does not care about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the
status of traffic lights too.

Please note that our simulator also provides the exact location of
traffic lights and their current status in `/vehicle/traffic_lights`
message. You can use this message to build this node as well as to
verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.

'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        self.waypoints = []
        self.x_ave = 0.0
        self.y_ave = 0.0
        self.rotate = 0.0
        self.phi = []
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # There is no topic (yet) for /obstacle_waypoint
        #rospy.Subscriber('/obstacle_waypoint', ???, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # for debugging
        #self.fd = open("/home/student/vercap/indices.csv", "w")
        rospy.spin()
        
    def get_index(self, x, y):
        rho = self.get_angle(x, y)
        # special case of wrap around when past last waypoint
        if rho > self.phi[-1]:
            return 0
        
        idx = 0
        while rho > self.phi[idx]:
            idx += 1

        #self.fd.write("%f,%f,%d\n" % (x, y, idx))
        return idx

    def get_angle(self, x, y):
        # First center
        xc = x - self.x_ave
        yc = y - self.y_ave
        
        # and now rotate
        xr = xc * math.cos(self.rotate) - yc * math.sin(self.rotate)
        yr = yc * math.cos(self.rotate) + xc * math.sin(self.rotate)
        
        # rho now starts at 0 and goes to 2pi for the track waypoints
        rho = math.pi - math.atan2(xr, yr)

        if rho > 6.28:
            rospy.logwarn("large rho %f" % rho)
            
        return rho

    def pose_cb(self, msg):
        # pose_cb might be called before waypoints_cb
        if self.waypoints == []:
            return None

        idx = self.get_index(msg.pose.position.x, msg.pose.position.y)            
        self.publish(idx)

    def waypoints_cb(self, lane):
        wp = lane.waypoints
        x_tot = 0.0
        y_tot = 0.0
        for p in wp:
            x_tot += p.pose.pose.position.x
            y_tot += p.pose.pose.position.y

        # We use the average values to recenter the waypoints
        self.x_ave = x_tot / len(wp)
        self.y_ave = y_tot / len(wp)
        
        # The very first waypoint determines the angle we need to rotate
        # all waypoints by
        xc = wp[0].pose.pose.position.x - self.x_ave
        yc = wp[0].pose.pose.position.y - self.y_ave
        self.rotate = math.atan2(xc, yc) + math.pi

        for p in wp:
            rho = self.get_angle(p.pose.pose.position.x, p.pose.pose.position.y)
            self.phi.append(rho)
                
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
        lane.header.frame_id = '/world'
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
