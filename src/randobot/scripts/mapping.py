#!/usr/bin/env python
import rospy
import math
import numpy as np
import random
from nav_msgs.msg import OccupancyGrid, GridCells, Odometry
from geometry_msgs.msg import  Point, PoseStamped
from sklearn.cluster import MeanShift, estimate_bandwidth
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient, GoalStatus

# initialize globals
grid_msg = GridCells()
cluster_msg = GridCells()
seenMap = False
grid_map = [[]]
seenOdom = False
cluster_list = []
frontier_list = []
hasMoved = False
success = True
mapComplete = False
avail_centers = []



def odomCallback(msg):
    #use global variables
    global curr_x, curr_y, seenOdom
    #update position and orientation
    curr_x = msg.pose.pose.position.x
    curr_y = msg.pose.pose.position.y
    seenOdom = True

def sendNewPathGoal(clusters):
    # publish new goal based on frontier and clusters
    global success, sent_pose, hasMoved, map_x, map_y, resolution, origin_x, origin_y
    # print clusters
    if not len(clusters) < 1:
        hasMoved = True
        smallest_pose = 0
        smallest_dist = math.sqrt((clusters[0][0] - origin_x)**2 + (clusters[0][1] - origin_y)**2)
        i = 0
        # find the cluster center that is closest to you
        for cluster in clusters:
            if not cluster == []:
                this_dist = math.sqrt((cluster[0] - origin_x)**2 + (cluster[1] - origin_y)**2)
                if this_dist < smallest_dist:
                    if not success:
                        if cluster[0] == sent_pose[0] and cluster[1] == sent_pose[1]:
                            clusters.pop(i)
                        else:
                            smallest_pose = i
                    else:
                        smallest_pose = i
                i += 1
        # send the closest cluster to the robot
        sent_pose = clusters.pop(smallest_pose)
        # send goal pose to nav stack
        print 'Sending Node: ', sent_pose
        MoveRobot(sent_pose)

def MoveRobot(destination):
    print 'moving to ', destination
    goal = MoveBaseGoal()
    pose_msg = PoseStamped()
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "/map"
    pose_msg.pose.position.x = (destination[0] * resolution) + map_x
    pose_msg.pose.position.y = (destination[1] * resolution) + map_y
    pose_msg.pose.position.z = 0
    quaternion = quaternion_from_euler(0,0,3.14)
    pose_msg.pose.orientation.x = quaternion[0]
    pose_msg.pose.orientation.y = quaternion[1]
    pose_msg.pose.orientation.z = quaternion[2]
    pose_msg.pose.orientation.w = quaternion[3]
    goal.target_pose = pose_msg
    move_base_client.send_goal(goal)

def findNearest(val, start_point_x, start_point_y, stop_at_first):
    global grid_map
    open_set = [[int(start_point_x), int(start_point_y)]]
    # print open_set
    closed_set = []
    return_set = []
    while not open_set == []:
        curr_point = open_set.pop(0)
        closed_set.append(curr_point)
        for x_offset in (-1, 0, 1):
            for y_offset in (-1, 0, 1):
                my_x = int(curr_point[0] + x_offset)
                my_y = int(curr_point[1] + y_offset)
                my_graph_val = grid_map[my_x][my_y]
                my_point = [my_x, my_y]
                if my_graph_val == val:
                    if stop_at_first == True:
                        return my_point
                    else:
                        return_set.append(my_point)
                elif my_graph_val <= 0:
                    if my_point not in open_set and my_point not in closed_set:
                        open_set.append(my_point)
    displayPoints(return_set, closed_set)
    return return_set


def mapCallback(msg):
    global resolution, curr_y, curr_x, map_x, map_y, grid_map, seenMap, seenOdom, origin_x, origin_y
    # get data from map message
    if seenOdom:
        resolution = (msg.info.resolution)
        map_x = msg.info.origin.position.x
        map_y = msg.info.origin.position.y
        origin_x = int((curr_x - map_x)//resolution)
        origin_y = int((curr_y - map_y)//resolution)
        #create graph with map data
        width, height = msg.info.width, msg.info.height;
        #remake map graph
        grid_map = [[-2 for x in range(height)] for y in range(width)]
        last_data = -1
        #go through data and find the frontier
        for i in range(0 , len(msg.data)-1):
            this_data = msg.data[i]
            # locate frontier
            i_x = i%width
            i_y = i//width
            grid_map[i_x][i_y] = this_data
            if this_data > 0:
                for j in (-1,0,1):
                    for h in (-1,0,1):
                        my_x = i_x + j
                        my_y = i_y + h
                        grid_map[my_x][my_y] = 100
        seenMap = True

def makeFrontier():
    global origin_x, origin_y, grid_map, avail_centers
    avail_centers = []
    frontier_set = [[]]
    frontier_set = findNearest(-1, origin_x, origin_y, False)
    front = []
    print 'updated Frontier, length = ', len(frontier_set)
    # if frontier set is small enough that a cluster cant be formed, the map
    # is sufficiently completed. any leftover points are usually noise or
    # already being finished
    if len(frontier_set) >= 10:
        # remove the first empty set in the list (initialized but not helpful)
        frontier_set.pop(0)
        front = np.reshape(frontier_set, (len(frontier_set), -1))
        # use sklearn function to cluster frontier data
        centers = []
        bandwidth = estimate_bandwidth(front, n_samples=10)
        mean_shift_cluster = MeanShift(bandwidth=bandwidth)
        mean_shift_cluster.fit(front)
        labels = mean_shift_cluster.labels_
        centers = mean_shift_cluster.cluster_centers_
        unique_labels = np.unique(labels)
        num_clusters = len(unique_labels)
        # adjust centers of clusters to be in reachable locations
        for center_point in centers:
            avail_centers.append(findNearest(0, center_point[0], center_point[1], True))
    else:
        avail_centers = frontier_set
        if avail_centers < 3:
            mapComplete = True
            print 'Map complete!!!!'


def displayPoints(frontier, peaks):
    global map_x, map_y, resolution
    point_list = []
    current_list = frontier
    publish_topic = nodePub
    # print both frontier set and clusters
    for x in range(2):
        for this_point in current_list:
            if not this_point == []:
                point_msg = Point()
                # convert to real world units
                point_msg.x = map_x + ((this_point[0] ) * resolution)
                point_msg.y = map_y + ((this_point[1] ) * resolution)
                point_msg.z = 0
                # create list of points
                point_list.append(point_msg)
        # publish grid cells to RVIZ
        grid_msg.header.frame_id = '/map'
        grid_msg.cells = point_list
        grid_msg.cell_width = resolution
        grid_msg.cell_height = resolution
        publish_topic.publish(grid_msg)
        # switch which list youre publishing and repeat
        current_list = peaks
        publish_topic = clusterPub


def atGoal():
    global origin_x, origin_y, sent_pose, success
    # check if you are somewhat near the goal location
    if origin_x >= sent_pose[0] - 5 and origin_x <= sent_pose[0] + 5:
        if origin_y >= sent_pose[1] - 5 and origin_y <= sent_pose[1] + 5:
            print 'Within Tolerance'
            success = True
            return True
    else:
        return False


def needToWiggle():
    if move_base_client.get_state == GoalStatus.ABORTED or move_base_client.get_state == GoalStatus.REJECTED:
        return True
    else:
        return False


def Wiggle():
    global origin_x, origin_y
    print 'Attempting to Wiggle Robot'
    while needToWiggle(): # and not rospy.is_shutdown():
        # print "Abort detected"
        wiggle_x = origin_x + random.randint(-6,6)
        wiggle_y = origin_y + random.randint(-6,6)
        # manually tell the robot to move
        MoveRobot([wiggle_x, wiggle_y])
        # rospy.sleep(1)
        move_base_client.wait_for_result()


if __name__ == '__main__':
     try:
         rospy.init_node('turtle_mapping', anonymous=True)
         move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        #get new map msg
         mapSub = rospy.Subscriber("/map", OccupancyGrid, mapCallback)
         #get new odom msg
         odomSub = rospy.Subscriber("/odom", Odometry, odomCallback)
         nodePub = rospy.Publisher("/searching", GridCells, queue_size = 10)
         clusterPub = rospy.Publisher("/clusters", GridCells, queue_size = 10)

         rospy.sleep(0.1)#short delay to ensure msgs have been received
         while not rospy.is_shutdown():
             # update the frontier then complete mapping until the map is complete
             if seenMap and not mapComplete:
                 makeFrontier()
                 # if not avail_centers == []:
                 sendNewPathGoal(avail_centers)
                 if hasMoved:
                     time = rospy.get_time()
                     # delay until you have reached your destination, failed
                     # to get to your destination, or timed out
                     while not atGoal():

                         # if the path was aborted or could not be reached or failed for
                         # whatever reason, send a random goal around you to try and wiggle
                         # free from your bad position.
                         if needToWiggle() or (rospy.get_time() > time + 10):
                             print 'Trying to wiggle'
                             Wiggle()
                             print 'Wiggle successful'
                             success = False
                             break
                         #
                         # if (rospy.get_time() > time + 10):
                         #     print 'Timed Out'
                         #     success = False
                         #     print 'Trying to wiggle'
                         #     Wiggle()
                         #     print 'Wiggle successful'
                         #     break

                         move_base_client.wait_for_result()
                         # if you couldnt make a valid path, move on
                         if move_base_client.get_state == GoalStatus.REJECTED:
                             break
                              # give the robot a few seconds to move a bit before
                              # recalculating the goal
                              # rospy.sleep(2)

         rospy.spin()#end if cntrl c is pressed

     except rospy.ROSInterruptException: pass
