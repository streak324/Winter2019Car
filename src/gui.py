#!/usr/bin/env python

import rospy
import os
import rospkg
import roslaunch
from std_msgs.msg import Int64

class GUI:
    def __init__(self, algoList):
        self.algoList = algoList
	
    def pub_selection(self, pub, choice):
   	#pub = rospy.Publisher('algo_selection', Int64, queue_size=10)
	print("\nThe selected algorithm is {}\n".format(choice))
	pub.publish(choice)

    def launch(self):
    	print("This is the main interface for switching between different algorithms on the autonomous car.\n")

	pub = rospy.Publisher('algo_selection', Int64, queue_size=10)
	
	choiceList = [num+1 for num in range(len(self.algoList))]

	while True:
	    print("Please select an algorithm to run on the car or press 'CTRL+C' to quit:\n")
    	    try:
            	for i, algo in enumerate(self.algoList, 1):
	            print("{0}: {1}\n".format(i, algo))
	    	choice = input()
		if choice in choiceList:
		    try:
		    	self.pub_selection(pub, choice)
            	    except rospy.ROSInterruptException:
		    	print("Publisher interrupted.\n")
		else:
		    print("\nInput only numbers corresponding to the algorithms.\n")
    	    except Exception:
		#print(e)
		print("\nInput only numbers.\n")

if __name__ == '__main__':
    rospy.init_node('gui', anonymous=True)    
    
    algoList = ['Follow the Gap Method', 'Vector Field Histogram', 'Vector Polar Histogram']
    
    gui = GUI(algoList)
    gui.launch()
    
    rospy.spin()


