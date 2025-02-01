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

def callBack(data):     #updates Frontier Points_Unexplored
    global frontiers
    frontiers=[]
    for point in data.points:
        frontiers.append(array([point.x,point.y]))

def mapCallBack(data):    #Updates the occupancy grid map
    global mapData
    mapData=data

# Node----------------------------------------------
def node():
    global frontiers,mapData,global1,global2,global3,globalmaps
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
                continue
            else:
                rospy.loginfo("Charging... %.1f seconds remaining" % 
                            (charging_time - (time() - charge_start_time)))
                continue
        
        # Check if need to charge
        if battery_level <= min_battery and not is_charging:
            rospy.loginfo("Low battery! Moving to charging station")
            for i in range(n_robots):
                robots[i].sendGoal(charging_station)
                if norm(robots[i].getPosition() - charging_station) < 0.5:
                    is_charging = True
                    charge_start_time = time()
                    rospy.loginfo("Started charging")
            continue
            
        # Normal exploration
        centroids = copy(frontiers)
        perform_exploration(robots, centroids, mapData, n_robots, info_radius, info_multiplier,
                           hysteresis_radius, hysteresis_gain, namespace, namespace_init_count,
                           delay_after_assignement)
        
        rate.sleep()

def perform_exploration(robots, centroids, mapData, n_robots, info_radius, info_multiplier, 
                       hysteresis_radius, hysteresis_gain, namespace, namespace_init_count, 
                       delay_after_assignement):
    """
    Performs the exploration logic for the robots
    
    Args:
        robots: List of robot objects
        centroids: List of frontier points
        mapData: Occupancy grid map data
        n_robots: Number of robots
        info_radius: Radius for information gain calculation
        info_multiplier: Multiplier for information gain
        hysteresis_radius: Radius for hysteresis
        hysteresis_gain: Gain for hysteresis
        namespace: Robot namespace
        namespace_init_count: Initial count for namespace
        delay_after_assignement: Delay after assigning goal
    """
    # Calculate information gain for each frontier point
    infoGain = []
    for ip in range(0, len(centroids)):
        infoGain.append(informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius))
    
    # Separate available and busy robots
    na = []  # available robots
    nb = []  # busy robots
    for i in range(0, n_robots):
        if (robots[i].getState() == 1):
            nb.append(i)
        else:
            na.append(i)
    rospy.loginfo("available robots: " + str(na))
    
    # Apply discount to information gain
    for i in nb + na:
        infoGain = discount(mapData, robots[i].assigned_point, centroids, infoGain, info_radius)
    
    revenue_record = []
    centroid_record = []
    id_record = []
    
    # Calculate revenue for available robots
    for ir in na:
        for ip in range(0, len(centroids)):
            cost = norm(robots[ir].getPosition() - centroids[ip])
            information_gain = infoGain[ip]
            if (norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius):
                information_gain *= hysteresis_gain
            
            revenue = information_gain * info_multiplier - cost
            revenue_record.append(revenue)
            centroid_record.append(centroids[ip])
            id_record.append(ir)
    
    # If no available robots, calculate revenue for busy robots
    if len(na) < 1:
        revenue_record = []
        centroid_record = []
        id_record = []
        for ir in nb:
            for ip in range(0, len(centroids)):
                cost = norm(robots[ir].getPosition() - centroids[ip])
                information_gain = infoGain[ip]
                if (norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius):
                    information_gain *= hysteresis_gain
                
                if ((norm(centroids[ip] - robots[ir].assigned_point)) < hysteresis_radius):
                    information_gain = informationGain(mapData, [centroids[ip][0], centroids[ip][1]], 
                                                     info_radius) * hysteresis_gain
                
                revenue = information_gain * info_multiplier - cost
                revenue_record.append(revenue)
                centroid_record.append(centroids[ip])
                id_record.append(ir)
    
    # Assign robot to best frontier
    if (len(id_record) > 0):
        winner_id = revenue_record.index(max(revenue_record))
        robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
        rospy.loginfo(namespace + str(namespace_init_count + id_record[winner_id]) + 
                     "  assigned to  " + str(centroid_record[winner_id]))
        rospy.sleep(delay_after_assignement)

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
