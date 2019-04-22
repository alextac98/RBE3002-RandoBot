#!/usr/bin/env python2.7

import rospy
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray


class Randobot:
    def __init__(self):
        rospy.init_node('randobot')
        rospy.Subscriber("/map", OccupancyGrid, self.mapcallback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.updategoal)

        #rospy.Publsiher("/move_base_simple/goal")
        
        self.isReady = True

    def generate_randompoint(self):
        # need to publish something

    def mapcallback(self, msg):
        self.current_map = [[0 for x in range(msg.info.width)] for y in range(msg.info.height)]

        x = 0 # width counter
        y = 0 # height counter
        for data in msg.data:
            if x is msg.info.width:
                x = 0
                y += 1
            self.current_map[x][y] = data

    def updategoal(self, msg):
        for status in msg.status_list and self.isReady:
            if status.status is not 3:
                self.isReady = False
                break
            if status.status is 3:
                self.isReady = True

        if self.isReady:
            print("At Goal")
        else:
            print("Not at Goal")
        

if __name__ == '__main__':
    Randobot()
    while not rospy.is_shutdown():

        rospy.spin()


# TODO:
# - Generate a random point whenever self.isReady is True
# - Check if random point is occupied in the map
# - send random point to the topic /move_base_simple/goal