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
stored_frontiers = []
last_exploration_point = None

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
    global frontiers,mapData,global1,global2,global3,globalmaps,stored_frontiers,last_exploration_point
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
    min_battery = 90
    battery_level = 100
    charging_time = 5.0  # 5 seconds for charging
    # Define multiple charging stations
    charging_stations = [
        array([-1.756932, -1.756932]),  # Original station
        array([1.5, 1.5]),              # New station 1
        array([-1.5, 1.5]),             # New station 2
        array([1.5, -1.5])              # New station 3
    ]
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
    
    while not rospy.is_shutdown():
        # Battery simulation
        current_time = time()
        elapsed_time = current_time - last_time
        last_time = current_time
        
        if not is_charging:
            battery_level -= drain_rate * elapsed_time
        
        for i in range(n_robots):
            current_pose = robots[i].getPosition()
            rospy.loginfo("-----------------------------")
            rospy.loginfo("Position: [%.2f, %.2f]" % (current_pose[0], current_pose[1]))
            rospy.loginfo("-----------------------------")
        
        rospy.loginfo("Battery Level: %.1f%%" % battery_level)
        
        # Check if charging
        if is_charging:
            if time() - charge_start_time >= charging_time:
                is_charging = False
                battery_level = 100  # Full charge
                rospy.loginfo("Charging complete! Battery at 100%")
                
                # Try to return to last exploration point
                if last_exploration_point is not None:
                    rospy.loginfo("Attempting to return to last exploration point: %s" % str(last_exploration_point))
                    for i in range(n_robots):
                        robots[i].sendGoal(last_exploration_point)
                        robots[i].assigned_point = None  # Reset assigned point to allow new assignments
                    
                    # Give some time to try reaching the point
                    attempts = 0
                    while attempts < 20 and not rospy.is_shutdown():  # Limit attempts
                        for i in range(n_robots):
                            current_pos = robots[i].getPosition()
                            distance_to_point = norm(current_pos - last_exploration_point)
                            rospy.loginfo("Robot %d - Distance to last exploration point: %.2f" % (i, distance_to_point))
                            
                            if distance_to_point <= 0.5:
                                rospy.loginfo("Robot %d reached last exploration point!" % i)
                                break
                        
                        attempts += 1
                        rate.sleep()
                
                # Resume exploration with proper frontier assignment
                rospy.loginfo("Resuming frontier exploration...")
                if len(frontiers) > 0:
                    centroids = copy(frontiers)
                    
                    # Reset robots for new assignments
                    for i in range(n_robots):
                        robots[i].assigned_point = None
                    
                    # Use the same assignment logic as main exploration loop
                    na = []  # available robots
                    nb = []  # busy robots
                    for i in range(0, n_robots):
                        if (robots[i].getState() == 1):
                            nb.append(i)
                        else:
                            na.append(i)
                    rospy.loginfo("Available robots: " + str(na))
                    
                    # Calculate info gain for all frontiers
                    infoGain = []
                    for ip in range(0, len(centroids)):
                        infoGain.append(informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius))
                    
                    # Apply discount based on robot assignments
                    for i in nb + na:
                        infoGain = discount(mapData, robots[i].assigned_point, centroids, infoGain, info_radius)
                    
                    revenue_record = []
                    centroid_record = []
                    id_record = []
                    
                    # Calculate revenues for available robots
                    for ir in na:
                        for ip in range(0, len(centroids)):
                            cost = norm(robots[ir].getPosition() - centroids[ip])
                            information_gain = infoGain[ip]
                            if norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius:
                                information_gain *= hysteresis_gain
                            revenue = information_gain * info_multiplier - cost
                            revenue_record.append(revenue)
                            centroid_record.append(centroids[ip])
                            id_record.append(ir)
                    
                    # Assign frontiers to robots
                    if len(revenue_record) > 0:
                        winner_id = revenue_record.index(max(revenue_record))
                        robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
                        rospy.loginfo(namespace + str(namespace_init_count + id_record[winner_id]) + 
                                    " assigned to " + str(centroid_record[winner_id]))
                        rospy.sleep(delay_after_assignement)
                
                # Reset exploration variables
                last_exploration_point = None
                stored_frontiers = []
                
            else:
                rospy.loginfo("Charging... %.1f seconds remaining" % 
                            (charging_time - (time() - charge_start_time)))
            rate.sleep()
            continue
        
        # Check if need to charge
        if battery_level <= min_battery and not is_charging:
            rospy.loginfo("Low battery! Storing current exploration state")
            stored_frontiers = copy(frontiers)
            for i in range(n_robots):
                if robots[i].assigned_point is not None:
                    last_exploration_point = copy(robots[i].assigned_point)
                else:
                    last_exploration_point = copy(robots[i].getPosition())
            rospy.loginfo("Stored last exploration point: %s" % str(last_exploration_point))
            
            # Find nearest charging station for each robot
            for i in range(n_robots):
                robot_pos = robots[i].getPosition()
                # Calculate distances to all charging stations
                distances = [norm(robot_pos - station) for station in charging_stations]
                
                # Print distances to all charging stations
                rospy.loginfo("Robot %d distances to charging stations:" % i)
                for station_idx, distance in enumerate(distances):
                    rospy.loginfo("Station %d at %s: %.2f meters" % 
                                (station_idx, str(charging_stations[station_idx]), distance))
                
                # Find the nearest station
                nearest_station_idx = distances.index(min(distances))
                nearest_station = charging_stations[nearest_station_idx]
                
                rospy.loginfo("Robot %d moving to nearest charging station %d at %s (distance: %.2f meters)" % 
                             (i, nearest_station_idx, str(nearest_station), min(distances)))
                
                # Send goal and verify it was sent
                robots[i].sendGoal(nearest_station)
                rospy.loginfo("Goal sent to robot %d. Current position: %s, Target: %s" % 
                             (i, str(robots[i].getPosition()), str(nearest_station)))
                
                # Wait for robot to reach charging station with continuous monitoring
                while not rospy.is_shutdown():
                    current_pos = robots[i].getPosition()
                    distance_to_station = norm(current_pos - nearest_station)
                    rospy.loginfo("Robot %d - Distance to station: %.2f, Current pos: %s, Target: %s" % 
                                 (i, distance_to_station, str(current_pos), str(nearest_station)))
                    
                    if distance_to_station <= 0.5:
                        rospy.loginfo("Robot %d reached charging station!" % i)
                        is_charging = True
                        charge_start_time = time()
                        rospy.loginfo("Started charging at station %d" % nearest_station_idx)
                        break
                    
                    # Resend goal if robot seems stuck
                    if robots[i].getState() != 1:  # If robot is not moving
                        rospy.logwarn("Robot %d seems stuck, resending goal" % i)
                        robots[i].sendGoal(nearest_station)
                    
                    rate.sleep()
            
            if is_charging:
                rospy.loginfo("Successfully started charging")
            else:
                rospy.logwarn("Failed to reach charging station, retrying...")
                continue
            
            continue
            
        # Normal exploration code
        centroids = copy(frontiers)
        
        if len(centroids) > 0:
            rospy.loginfo("Number of frontiers available: %d" % len(centroids))
            infoGain = []
            for ip in range(0, len(centroids)):
                infoGain.append(informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius))
        
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
