#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from copy import deepcopy

import math

ONE_MPH = 0.44704
CREEP_VELOCITY = 1.5
CREEP_RANGE = 30

# Target velocity in meters per second.
TARGET_VELOCITY = ONE_MPH * 24.0

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

BRAKING_RANGE = 200 # This _must_ be smaller than lookahead


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        self.waypoints = []
        self.x_ave = 0.0
        self.y_ave = 0.0
        self.cos_rotate = 0.0
        self.sin_rotate = 0.0
        self.phi = []
        self.current_velocity = 0.0
        self.red_light = -1
        self.lap_count = 0
        self.lap_toggle = False
        
        max_velocity = float(rospy.get_param("/waypoint_loader/velocity")) / 3.6
        self.target_velocity = min(TARGET_VELOCITY, max_velocity)
        rospy.logwarn("target velocity %f" % self.target_velocity)
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        
        # There is no topic (yet) for /obstacle_waypoint
        #rospy.Subscriber('/obstacle_waypoint', ???, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        rospy.spin()

    def lap(self, rho):
        # Ugly but robust
        if rho < 0.2 and self.lap_toggle:
            self.lap_count += 1
            rospy.logwarn("Completed lap %d" % self.lap_count)
            self.lap_toggle = False
        elif rho > 6.2:
            self.lap_toggle = True
        return
    
    def get_index(self, x, y):
        rho = self.get_angle(x, y)
        # special case of wrap around when past last waypoint
        if not self.phi or rho > self.phi[-1]:
            return 0
        
        self.lap(rho)
        
        idx = 0
        while rho > self.phi[idx]:
            idx += 1
        return idx

    def get_angle(self, x, y):
        # First center
        xc = x - self.x_ave
        yc = y - self.y_ave
        
        # and now rotate
        xr = xc * self.cos_rotate - yc * self.sin_rotate
        yr = yc * self.cos_rotate + xc * self.sin_rotate
        
        # rho now starts at 0 and goes to 2pi for the track waypoints
        rho = math.pi - math.atan2(xr, yr)
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
        rot_angle = math.atan2(xc, yc) + math.pi
        self.cos_rotate = math.cos(rot_angle)
        self.sin_rotate = math.sin(rot_angle)

        for p in wp:
            rho = self.get_angle(p.pose.pose.position.x, p.pose.pose.position.y)
            self.phi.append(rho)
                
        self.waypoints.extend(wp)
        # make wrap around easier by extending waypoints
        self.waypoints.extend(wp[0:LOOKAHEAD_WPS])

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
        return

    def traffic_cb(self, msg):
        self.red_light = msg.data
        return
    
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        if not (0 <= waypoint <= len(waypoints)):
            rospy.logwarn("WTF: %d %d" % (waypoint, len(waypoints)))
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
        # it looks like we sometimes get bad red_light data
        dist = self.red_light - idx
        if dist > 0 and self.red_light > -1 and dist < BRAKING_RANGE:
            if idx < 280 and dist < CREEP_RANGE and self.current_velocity < CREEP_VELOCITY:
                for i in range(CREEP_RANGE - 5):
                    self.set_waypoint_velocity(lane.waypoints, i, CREEP_VELOCITY)
                for i in range(CREEP_RANGE - 5, LOOKAHEAD_WPS):
                    self.set_waypoint_velocity(lane.waypoints, i, 0.0)
            else:                
                incr = 1.1 * (self.current_velocity / max(0.1, float(dist)))
                velo = self.current_velocity - incr
                for i in range(dist):
                    self.set_waypoint_velocity(lane.waypoints, i, max(0.0, velo))
                    velo -= incr
                #rospy.logwarn("%d  %d  %d  %d" % (dist, len(lane.waypoints), LOOKAHEAD_WPS))
                for i in range(dist, LOOKAHEAD_WPS):
                    self.set_waypoint_velocity(lane.waypoints, i, 0.0)
        else:
            for i in range(LOOKAHEAD_WPS):
                self.set_waypoint_velocity(lane.waypoints, i, self.target_velocity)

        self.final_waypoints_pub.publish(lane)
        

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
