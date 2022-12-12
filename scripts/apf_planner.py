#! /usr/bin/env python3

from cmath import inf
from visualization_msgs.msg import Marker,MarkerArray
# import roslib, sys, rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Point,PoseStamped, Twist
import numpy as np
import math
import time

     
def computeRepulsiveForce(laser_scan):

    eta = 10 # scaling factor for repulsive force
    q_star = 5 # threshold distance for obstacles
    laser_data = laser_scan
    ranges = np.array(laser_data.ranges)
    angle = laser_data.angle_min
    resolution = laser_data.angle_increment
    vector_sum = np.array([0.0,0.0])
    
    # The lidar outputs 360 degree data and therefore we get data of 360 points within the range of the sensor.
    # Each obstacle detected by the sensor will occupy some points out of these 360 points. We treat each single point as 
    # an individual obstacle.
    for distance in ranges:
        if(distance<q_star):
            mag = np.absolute(eta*(1/q_star - 1/distance)*(1/distance**2))
        else:
            mag = 0
        
        # This is the negative gradient direction
        vector = -1*np.array([np.cos(angle),np.sin(angle)])
        vector *= mag
        vector_sum += vector
        angle += resolution

    # Normalization of the repulsive forces
    return np.array([vector_sum[0],vector_sum[1]])*(1/len(ranges))
#------------------------------------------

def computeAttractiveForce(odom, goal_pose, position_all):

    odom_data = odom
    pos_x = odom_data.pose.pose.position.x
    pos_y = odom_data.pose.pose.position.y
    pos = []
    pos.append(pos_x)
    pos.append(pos_y)
    zeta = 2 # scaling factor for attractive force
    d_star = 3 # threshoild distance for goal

    closest_waypoint = []
    closest_waypoint = getClosestWaypoint(pos, position_all)

    dist_to_goal = np.sqrt((pos_x - goal_pose.pose.position.x)**2 + (pos_y - goal_pose.pose.position.y)**2)

    index_waypoint = closest_waypoint[0]

    # If the closest waypoint is the last point on the global path then direct the robot towards the goal position
    if(index_waypoint == len(position_all)-1):
        angle = np.arctan2(goal_pose.pose.position.y - pos_y, goal_pose.pose.position.x - pos_x)
        if (dist_to_goal<=d_star):
            mag = np.absolute(zeta*dist_to_goal)
        else :
            mag = d_star*zeta
        vector = 1*np.array([np.cos(angle), np.sin(angle)])
        vector *= mag
        return np.array([vector[0],vector[1]])

    # We use a lookahead distance concept in order to drive the robot. Feel free to customize this block of code.
    # The robot follows the waypoint which is 10 points ahead of the closest waypoint. If the robot is within 10 points
    # from the goal ,use the goal position instead to drive the robot.
    if(index_waypoint + 10 <len(position_all)-2):
        pt_next = position_all[index_waypoint + 10]
        angle = np.arctan2(pt_next[1]-pos_y,pt_next[0]-pos_x)
    else:
        angle = np.arctan2(goal_pose.pose.position.y - pos_y , goal_pose.pose.position.x - pos_x)

    if (dist_to_goal<=d_star):
        mag = np.absolute(zeta*dist_to_goal)
    else :
        mag = d_star*zeta

    vector = 1*np.array([np.cos(angle), np.sin(angle)])
    vector *= mag
    return np.array([vector[0],vector[1]])
#------------------------------------------

def getClosestWaypoint(point, points):
    i=0
    pt=[]
    dist = math.inf
    for p in points:
        if(math.dist(p,point)<dist):
            dist = math.dist(p,point)
            pt = p
            i = points.index(pt)
    return [i,pt]

#------------------------------------------

def handleGlobalPlan(global_path):
    position_x = []
    position_y = []
    i=0
    print (np.shape(global_path.poses))
    while(i <= len(global_path.poses)-1):
        position_x.append(global_path.poses[i].pose.position.x)
        position_y.append(global_path.poses[i].pose.position.y)
        i=i+1
    position_all = [list(double) for double in zip(position_x,position_y)]
    
    return position_all


def computeVelocity(obstacle_vector):
    cmd = Twist()
    min_vel_x = -1
    max_vec_x = 1
    min_vel_y = -1
    max_vec_y = 1
    drive_scale = 0.1 # scaling factor to scale the net force

    if (obstacle_vector is None):
        return cmd
        
    else:
        # We use velocity based potential field,that is, the gradient/force is directly commanded as velocities
        # instead of force or acceleration. 

        vel_x = obstacle_vector[0] * drive_scale
        vel_y = obstacle_vector[1] * drive_scale
        cmd.linear.x = np.clip(vel_x, min_vel_x, max_vec_x)
        cmd.linear.y = np.clip(vel_y, min_vel_y, max_vec_y)

    return cmd


def getVelocity(laser_scan, odom, goal_pose, global_plan):
    position_all = handleGlobalPlan(global_plan)
    net_force = computeRepulsiveForce(laser_scan) + computeAttractiveForce(odom, goal_pose, position_all)
    vector = Point(net_force[0],net_force[1], 0)
    obstacle_vector = np.array([vector.x, vector.y])
    cmd_vel  = computeVelocity(obstacle_vector)
    return cmd_vel
