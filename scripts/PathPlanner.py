#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import numpy as np

#Import message type, OccupancyGrid is the message type
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import tf2_ros
import matplotlib.pyplot as plt


odom=[0,0]
rgoal=[0,0]
readgoal=False
def goal_cb(data):#ゴール受信関数
	global readgoal
	readgoal=True
	rgoal[0]=data.pose.position.x
	rgoal[1]=data.pose.position.y

#ROS初期設定
rospy.init_node('PathPlanner')
path_pub = rospy.Publisher('/path', Path, queue_size=10)
goal_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped, goal_cb)
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

#path generation
def path_generation(init,sPath,w,h,resolution):
	#Initialize odometry header
	global path_pub
	global head
	path = Path()
	path_header = Header()
	path_header.seq = 0
	path_header.stamp = rospy.Time.now()
	path_header.frame_id = "map"
	hh=h/2
	hw=w/2

	temp_pose = PoseStamped()
	temp_pose.pose.position.x = (init[0]-hh)*resolution
	temp_pose.pose.position.y = (init[1]-hw)*resolution
	temp_pose.pose.position.z = 0
	temp_pose.header = path_header
	temp_pose.header.seq = 0
	path.poses.append(temp_pose)
	for i in range(0,len(sPath)):
		temp_pose = PoseStamped()
		temp_pose.pose.position.x = (sPath[i][0]-hh)*resolution
		temp_pose.pose.position.y = (sPath[i][1]-hw)*resolution
		temp_pose.pose.position.z = 0
		temp_pose.header = path_header
		temp_pose.header.seq = i+1
		path.poses.append(temp_pose)
	#print path.poses
	path.header = path_header
	path_pub.publish(path)
	rospy.loginfo('End path generation')


#path planner
import a_star

def a_star_pathplanner(start,goal,grid):
	test_planner = a_star.PathPlanner(grid,False)
	init,path=test_planner.a_star(start,goal)
	print init
	print path
	return path ,init

#Map
class Map(object):
	def __init__(self):
	#rospySubscribe to the map topic, the second is the data type, the third is the callback function
	# Pass the subscribed data to the callback function, which is the mapmsg variable
	#If a topic comes, call the callback function directly
		self.map_sub = rospy.Subscriber("map",OccupancyGrid, self.callback)
		print "get map~"
		#The output below is the address, not the data
		print self.map_sub

	#The definition of the callback function, passed mapmsg
	def callback(self,mapmsg):
		#obstacleList = np.empty(3)
		try:
			print "into callback"
			#Mainly want to get data, here is stored map information
			map = mapmsg.data
			# Below is the tuple type
			print type(map)
			#Change to numpy format that can draw pictures
			map = np.array(map)
			#The following output is (368466,), obviously can not draw
			print map.shape
			#Need to reshape, factor the above numbers online, then calculate the two largest factors
			#So it's probably like this:
			map = map.reshape((mapmsg.info.height,mapmsg.info.width))
			print map
			#You can see that most of the values ​​are -1, so you need to regularize the values
			row,col = map.shape
			print row,col
			tem = np.zeros((row,col))
			grid = np.zeros((row,col))
			for i in range(row):
				for j in range(col):
					grid[j,i]=0
					if(map[i,j]==-1):
						tem[j,i]=255
					else:
						tem[j,i]=map[i,j]
						if map[i,j]==100:
							grid[j,i]=1
							#obstacleList=np.vstack((obstacleList, np.array([j,i,0.15])))

			print map.shape
			#obstacleList=np.delete(obstacleList, 0, 0)

		except Exception,e:
			print e
			rospy.loginfo('convert rgb image error')

		#print obstacleList
		#plt.imshow(grid)
		#plt.show()
		#rrt_pathplanner(obstacleList.tolist())
		global readgoal
		rospy.loginfo('Map conversion completed')
		while not rospy.is_shutdown():
			if readgoal:#ゴールを受信
				rospy.loginfo('Receive goal')
				#Robotの座標取得
				try:
					t = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
					odom[0]=t.transform.translation.x
					odom[1]=t.transform.translation.y
				except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
					rospy.loginfo(e)
				print "start",odom,"goal",rgoal
				#座標変換
				start=[int((odom[1]/mapmsg.info.resolution+(mapmsg.info.height/2))),int((odom[0]/mapmsg.info.resolution+(mapmsg.info.width/2)))]
				goal=[int((rgoal[1]/mapmsg.info.resolution+(mapmsg.info.height/2))),int((rgoal[0]/mapmsg.info.resolution+(mapmsg.info.width/2)))]
				#経路計算
				path,init=a_star_pathplanner(start,goal,grid.tolist())
				#経路配信
				path_generation(init,path,mapmsg.info.height,mapmsg.info.width,mapmsg.info.resolution)
				readgoal=False

	def getImage():
		return self.rgb_image


def main(_):
	v=Map()
	rospy.spin()

if __name__=='__main__':
	main('_')