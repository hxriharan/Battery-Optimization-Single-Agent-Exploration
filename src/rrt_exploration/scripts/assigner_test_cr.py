#!/usr/bin/env python

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from rrt_exploration.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot,informationGain,discount
from numpy.linalg import norm

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]

robot_battery_levels = {}
robot_charging_states={}
robot_saved_positions={}
robot_charge_start_times={}
robot_returning_from_charge={} #track Post charging states


def callBack(data):
    global frontiers
    frontiers=[]
    for point in data.points:
        frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data

# Node----------------------------------------------
def node():
    global frontiers,mapData,global1,global2,global3,globalmaps
    global robot_battery_levels,robot_charging_states,robot_saved_positions
    global robot_saved_frontiers,robot_charge_start_times,robot_returning_from_charge
    rospy.init_node('assigner_test', anonymous=False)
    
    # fetching all parameters
    map_topic= rospy.get_param('~map_topic','/map')
    info_radius= rospy.get_param('~info_radius',1.0)
    info_multiplier=rospy.get_param('~info_multiplier',3.0)        
    hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)
    hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)
    frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')    
    n_robots = rospy.get_param('~n_robots',1)
    namespace = rospy.get_param('~namespace','')
    namespace_init_count = rospy.get_param('namespace_init_count',1)
    delay_after_assignement=rospy.get_param('~delay_after_assignement',0.5)
    rateHz = rospy.get_param('~rate',100)
    
    # Battery and charging parameters
    drain_rate = 0.1
    min_battery = 97
    battery_level = 100
    charging_time = 5.0  # 3 seconds for charging
    charging_station = array([-0.280000, 0.610000])  # Change these coordinates as needed
    is_charging = False
    charge_start_time = None
    
    rate = rospy.Rate(rateHz)
    
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, callBack)
    
    while len(frontiers)<1:
        pass
    centroids=copy(frontiers)    
    
    while (len(mapData.data)<1):
        pass

    robots=[]
    if len(namespace)>0:
        for i in range(0,n_robots):
            robots.append(robot(namespace+str(i+namespace_init_count)))
    elif len(namespace)==0:
            robots.append(robot(namespace))
    
    for i in range(0,n_robots):
        robots[i].sendGoal(robots[i].getPosition())

    last_time = time()

    for i in range(n_robots):
        robot_battery_levels[i] = 100
        robot_charging_states[i] = False
        robot_saved_positions[i] = None
        robot_saved_frontiers[i] = None
        robot_charge_start_times[i] = None
        robot_returning_from_charge[i] = False
    
    while not rospy.is_shutdown():
        current_time = time()
        elapsed_time = current_time - last_time
        last_time = current_time

        # Process each robot
        for i in range(n_robots):
            current_pose = robots[i].getPosition()
            
            # Update battery for non-charging robots
            if not robot_charging_states[i]:
                robot_battery_levels[i] -= drain_rate * elapsed_time
            
            rospy.loginfo("Robot %d - Position: [%.2f, %.2f], Battery: %.1f%%" % 
                         (i, current_pose[0], current_pose[1], robot_battery_levels[i]))

            # Handle charging completion
            if robot_charging_states[i]:
                if time() - robot_charge_start_times[i] >= charging_time:
                    robot_charging_states[i] = False
                    robot_battery_levels[i] = 100
                    robot_returning_from_charge[i] = True
                    rospy.loginfo("Robot %d charging complete! Returning to saved position" % i)
                    
                    if robot_saved_positions[i] is not None:
                        robots[i].sendGoal(robot_saved_positions[i])
                    continue
                else:
                    rospy.loginfo("Robot %d charging... %.1f seconds remaining" % 
                                (i, charging_time - (time() - robot_charge_start_times[i])))
                    continue

            # Handle return to saved position
            if robot_returning_from_charge[i]:
                if robot_saved_positions[i] is not None:
                    if norm(current_pose - robot_saved_positions[i]) < 0.5:
                        rospy.loginfo("Robot %d reached saved position" % i)
                        robot_returning_from_charge[i] = False
                        if robot_saved_frontiers[i] is not None:
                            robots[i].assigned_point = robot_saved_frontiers[i]
                        robot_saved_positions[i] = None
                        robot_saved_frontiers[i] = None
                    continue
                else:
                    robot_returning_from_charge[i] = False

            # Check if need to charge
            if robot_battery_levels[i] <= min_battery and not robot_charging_states[i]:
                rospy.loginfo("Robot %d low battery! Saving position and moving to charging station" % i)
                robot_saved_positions[i] = current_pose.copy()
                robot_saved_frontiers[i] = robots[i].assigned_point.copy() if robots[i].assigned_point is not None else None    
                robots[i].sendGoal(charging_station)

                if norm(current_pose - charging_station) < 0.5:
                    robot_charging_states[i] = True
                    robot_charge_start_times[i] = time()
                    rospy.loginfo("Robot %d started charging" % i)
                continue

        # Only proceed with frontier assignment for available robots
        available_robots = [i for i in range(n_robots) 
                          if not robot_charging_states[i] and 
                          not robot_returning_from_charge[i]]
        
        if available_robots:
            centroids = copy(frontiers)

            infoGain=[]
            for ip in range(0,len(centroids)):
                infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))            
            na=[]
            nb=[]
            for i in range(0,n_robots):
                if (robots[i].getState()==1):
                    nb.append(i)
                else:
                    na.append(i)    
            rospy.loginfo("available robots: "+str(na))    
            
            for i in nb+na:
                infoGain=discount(mapData,robots[i].assigned_point,centroids,infoGain,info_radius)
                
            revenue_record=[]
            centroid_record=[]
            id_record=[]
            
            for ir in na:
                for ip in range(0,len(centroids)):
                    cost=norm(robots[ir].getPosition()-centroids[ip])        
                    threshold=1
                    information_gain=infoGain[ip]
                    if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
                        information_gain*=hysteresis_gain
                    
                    revenue=information_gain*info_multiplier-cost
                    revenue_record.append(revenue)
                    centroid_record.append(centroids[ip])
                    id_record.append(ir)
            
            if len(na)<1:
                revenue_record=[]
                centroid_record=[]
                id_record=[]
                for ir in nb:
                    for ip in range(0,len(centroids)):
                        cost=norm(robots[ir].getPosition()-centroids[ip])        
                        threshold=1
                        information_gain=infoGain[ip]
                        if (norm(robots[ir].getPosition()-centroids[ip])<=hysteresis_radius):
                            information_gain*=hysteresis_gain
                        
                        if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
                            information_gain=informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius)*hysteresis_gain

                        revenue=information_gain*info_multiplier-cost
                        revenue_record.append(revenue)
                        centroid_record.append(centroids[ip])
                        id_record.append(ir)
            
            if (len(id_record)>0):
                winner_id=revenue_record.index(max(revenue_record))
                robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
                rospy.loginfo(namespace+str(namespace_init_count+id_record[winner_id])+"  assigned to  "+str(centroid_record[winner_id]))    
                rospy.sleep(delay_after_assignement)
                
        rate.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
