#!/usr/bin/env python2
# racecar AI for Driver

import rospy
import math
from race.msg import drive_param
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32

import constants

import time

def dist_between(pos1, pos2):
    return math.sqrt( (pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 )

class Car:
    def __init__(self):
        self.lidar = []
        self.motorSpeed = 0
        self.turnAngle = 0
        self.velocity = 0.0
        self.fricCoeff = 10 # coefficient of friction between ground and tires
        self.slowdown_distance = 2
        self.carLength = 0.5 # length of car determines smallest turn radius
        self.reading_number = 1 # ray casts per degree (2 = 360 readings)
        self.speed_factor = 0.4 # car's speed limit: range 0.2 (min) to 1
        self.lidar = [10.0]*(180*self.reading_number+1) #want lidar range to be 0 to 180 inclusive
        self.init_time = rospy.get_time()
        self.motorKill = False;
        self.stopTime = 0; # timestamp for breaking
        self.alert = "none"
	self.angular_precision = math.pi / (len(self.lidar))
        self.back_right = 0
        self.back_left = 0
        #self.angular_precision =  0.25 * math.pi /180

    def get_dur(self):
        return str("%.3f" % (rospy.get_time() - self.init_time))
    
    def changeMotorSpeed(self, val):
        # print("requested motorSpeed: " + str(100*val/2.0))
        self.velocity = val
        if(self.velocity > self.speed_factor): # cropping motor speed here just for now
            self.velocity = self.speed_factor

        if(self.motorKill):
            # print("[" + self.get_dur() + "] Alert: motors killed")
            self.alert = "Motors killed"
            #if(self.stopTime > time.time() and self.motorSpeed != 0):
            #    self.velocity = -1
                # print("-50 motor speed")
            #else:
                # print("0 motor speed")
               # self.velocity = 0
            self.velocity = 0
        self.motorSpeed = 100*self.velocity/2.0

    def changeTurnAngle(self, ang):
        # clip turn angle to +/- pi/4
        if (ang > math.pi/4):
            self.turnAngle = math.pi/4
            return
        if (ang < -math.pi/4):
            self.turnAngle = -math.pi/4
            return
        self.turnAngle = ang

    def __del__(self):
        self.changeMotorSpeed(0)
        self.changeTurnAngle(0)

class RacecarAI:
    '''This is the Driver module that controls the car's basic functions
    It is designed to operate with or without a codriver.
    Without a codriver, the driver will simply drive slowly and safely, avoiding
    obstances and performing turns when necessary. It is designed to be lightweight
    and free of hardcoding.
    '''

    def __init__(self, car):
        self.car = car
        self.fricCoeff = 10 # this will have to be adjusted to a realistic value
        self.safetyMode = False
        
        ''' Variables for Follow the Gap Method '''
        self.max_gap = -1 
        self.max_gap_start = -1
        self.max_gap_end = -1
        self.max_gapCenter = -1
        self.center_gap_index = -1
        self.obstacle_arry = []
        self.lidar_gap = [10.0]*(180*self.car.reading_number+1) 
        #self.goal_pos = [self.car._position[0],self.car._position[1]]
        self.goal_dist = 0
    #-------------------------------------------------------------------------------------
        
        
        self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10) # ros publisher
        self.pub2 = rospy.Publisher('alert', String, queue_size=10)
        self.pub4 = rospy.Publisher('motor', Float32, queue_size=10)
        self.pub5 = rospy.Publisher('angle', Float32, queue_size=10)
        

        self.listener()

    def kill_motors(self, data):
        print("Alert: motors killed")
        self.car.motorKill = True
        self.car.stopTime = time.time() + 0.5

    #-------------------------------------------------------------------------------------

    def _moveAwayFromWall(self, dist, angle):
        #if car is too close to a wall, modify angle to pull away
        left = min(self.car.lidar[0:20*self.car.reading_number])
        right = min(self.car.lidar[-20*self.car.reading_number:len(self.car.lidar)])
        # dist will be scaled by number of carLengths (currently 1x)
        if(left < dist and left < right):
            # print("[" + self.car.get_dur() + "] Alert: left " + str(left))
            self.alert = "left " + str(left)
            return angle + math.pi/8
        elif(right < dist and right < left):
            # print("[" + self.car.get_dur() + "] Alert: right "+ str(right))
            self.alert = "right " + str(right)
            return angle - math.pi/8
        else:
            self.alert = "none"
            return angle

    def _getFrontDist(self, angle=0):
        middle = len(self.car.lidar)//2
        # relative angle ranges from -pi/2 to pi/2
        lidar_beam_angle = (math.pi / len(self.car.lidar))
        #determine the range of values to scan
        scope = int(math.atan(1) / lidar_beam_angle) + 1
        # this converts relative angle to corresponding LIDAR index
        index = int(angle / lidar_beam_angle + middle)
        
        front_dist = self.car.lidar[index]
        for i in range(-scope,scope+1):
            if(self.car.lidar[index+i]*math.sin(abs(i)*lidar_beam_angle) < self.car.carLength/2):
                if(self.car.lidar[index+i] < front_dist):
                    front_dist = self.car.lidar[index+i]
        return front_dist

    def findGoalAngle(self):
        dist = self.dist_between(self.car._position,self.goal_pos)
        self.goal_dist = dist
        x_diff = self.goal_pos[0] - self.car._position[0]
        y_diff = self.goal_pos[1] - self.car._position[1]
        print(dist,x_diff,y_diff)
        if(x_diff == 0 and y_diff > 0):      # below car
            clicked_angle = math.pi/2
            clicked_angle_adjusted = clicked_angle
        elif(x_diff == 0 and y_diff < 0):    # above car
            clicked_angle = 3*math.pi/2
            clicked_angle_adjusted = clicked_angle
        elif(x_diff > 0 and y_diff == 0):    # right of car
            clicked_angle = 0
            clicked_angle_adjusted = clicked_angle
        elif(x_diff < 0 and y_diff == 0):    # left of car
            clicked_angle = math.pi
            clicked_angle_adjusted = clicked_angle
        elif(x_diff > 0 and y_diff < 0):    # Quadrant 1
            #print("Q1")
            clicked_angle = math.asin(y_diff/dist)
            clicked_angle_adjusted = 2*math.pi - clicked_angle
        elif(x_diff < 0 and y_diff < 0):    # Quadrant 2
            #print("Q2")
            clicked_angle = math.pi - math.asin(y_diff/dist)
            clicked_angle_adjusted = math.pi + clicked_angle
        elif(x_diff < 0 and y_diff > 0):    # Quadrant 3
            #print("Q3")
            clicked_angle = -(math.pi + math.asin(y_diff/dist))
            clicked_angle_adjusted = 2*math.pi + clicked_angle
        elif(x_diff > 0 and y_diff > 0):    # Quadrant 4
            #print("Q4")
            clicked_angle = -(3*math.pi/2 + math.asin(x_diff/dist))
            clicked_angle_adjusted = 2*math.pi + clicked_angle
        
        relative_angle = clicked_angle_adjusted - self.normalizeAngle(self.car._angle)
        #print("clicked_adj: ", clicked_angle_adjusted*180/math.pi)
        if(relative_angle >= 0 and relative_angle < math.pi/2) or (relative_angle <= 0 and relative_angle > -math.pi/2):
            #print("front")
            return clicked_angle_adjusted
        elif(relative_angle < math.pi and relative_angle > math.pi/2):
            #print("back right")
            return math.pi
        elif(relative_angle < 3*math.pi/2 and relative_angle > math.pi/2):
            #print("back left")
            return 0
        else:
            return math.pi
        
    def _findMaxGap(self):
        self.max_gap = -1
	del self.obstacle_arry[:]
        Threshold = 0.4 
        #scan_dist = 2.5
        scan_dist = 2
        num_gap = 0
        num_obstacles = 0
	#del self.lidar_gap [:]
        #print("------------------------------------------")
        #print(len(self.car.lidar))
        for i in range(0, len(self.car.lidar)-1):            # modifying array to show gap/no gap
            if(self.car.lidar[i] < scan_dist): 
                if((abs(self.car.lidar[i+1] - self.car.lidar[i])) > Threshold): # modify threshold or find a better way to distinguish between obstacles and gaps
                    self.lidar_gap[i] = 0   # gap
                else:
                    self.lidar_gap[i] = 1   # no gap
            else:
                #print("index i ", i) 
                self.lidar_gap[i] = 0
        self.car.lidar[len(self.car.lidar)-1] = 1
            #print(self.car.lidar[i])
            #if(i%10 == 0):
            #    print("\n")
            #print self.lidar_gap[i], 
        #print(self.lidar_gap[len(self.car.lidar)-1])
            #print(self.car.lidar[i])
            #print(self.car.lidar[i+1])
        #print("\n****************************************")
        #for i in range(0, len(self.car.lidar)-1):
        #   if(i%10 == 0):
        #       print("\n")
        #   print self.car.lidar[i],
        gapMap = {'gap': []}
        startGap = endGap = startObs = endObs = 0
        gap_width = 0
        for i in range(1, len(self.lidar_gap)):
            if self.lidar_gap[i-1] == 1 and self.lidar_gap[i] == 0:
                startGap = i
                if self.lidar_gap[i] == 0 and i == len(self.lidar_gap)-1:
                    endGap = startGap
                    gap_width = endGap - startGap + 1
                    gapMap['gap'].append((len(self.lidar_gap)-1, len(self.lidar_gap)-1, gap_width))
            elif self.lidar_gap[i-1] == 0 and self.lidar_gap[i] == 1:
                endGap = i-1
                gap_width = endGap - startGap + 1
                gapMap['gap'].append((startGap, endGap, gap_width))
            elif self.lidar_gap[i] == 0 and i == len(self.lidar_gap)-1:
                endGap = i
                gap_width = endGap - startGap + 1
                gapMap['gap'].append((startGap, len(self.lidar_gap)-1, gap_width))

        for i in range(1, len(self.lidar_gap)):
            if self.lidar_gap[i-1] == 0 and self.lidar_gap[i] == 1:
                startObs = i
                if self.lidar_gap[i] == 1 and i == len(self.lidar_gap)-1:
                    #gapMap['obs'].append((len(self.lidar_gap)-1, len(self.lidar_gap)-1))
                    self.obstacle_arry.append(self.car.lidar[i-1])
                    #gapMap['obs'].append(self.car.lidar[i-1])
            elif self.lidar_gap[i-1] == 1 and self.lidar_gap[i] == 0:
                endObs = i-1
                #gapMap['obs'].append((startObs, endObs))
                self.obstacle_arry.append(self.car.lidar[int((startObs+endObs)/2)])
                #gapMap['obs'].append(self.car.lidar[int((startObs+endObs)/2)])
            elif self.lidar_gap[i] == 1 and i == len(self.lidar_gap)-1:
                #gapMap['obs'].append((startObs, len(self.lidar_gap)-1))
                self.obstacle_arry.append(self.car.lidar[int((i+startObs)/2)])
                #gapMap['obs'].append(self.car.lidar[int((i+startObs)/2)])
        #print("\n")
        print(gapMap)
        self.max_gap = 0
        for gap in gapMap['gap']:
            #print(gap)
            if(gap[2] > self.max_gap):
                self.max_gap = gap[2]
                self.max_gap_start = gap[0]
                self.max_gap_end = gap[1]
        print(self.max_gap,self.max_gap_start,self.max_gap_end)
        
    def _findMaxGapCenter(self):
        center_angle = int(len(self.car.lidar)/2)
        #theta1 = self.car.angular_precision*(self.max_gap_start - center_angle)
        #theta2 = self.car.angular_precision*(self.max_gap_end - center_angle)
        #d1 = self.car.lidar[self.max_gap_start]
        #d2 = self.car.lidar[self.max_gap_end]
        #numerator = d1 + d2*math.cos(theta1+theta2)
        #denominator = math.sqrt(d1**2 + d2**2 + 2*d1*d2*math.cos(self.max_gap))
        #print(center_angle,theta1,theta2,d1,d2,numerator,denominator)
        #self.max_gapCenter = math.acos(numerator/denominator) - theta1
        self.center_gap_index = int((self.max_gap_end + self.max_gap_start) / 2)
        
        self.max_gapCenter = self.center_gap_index * self.car.angular_precision

        #print("gapCenter ",int((self.max_gap_end + self.max_gap_start) / 2))
        #self.screen.create_line(self.car._position[0], self.car._position[1],
        #                       self.car._position[0] + math.cos(self.max_gapCenter - math.pi/2 + self.car._angle)*100,
        #                       self.car._position[1] + math.sin(self.max_gapCenter - math.pi/2 + self.car._angle)*100, fill="white")

    def _findHeading(self):
        #angle = self.findGoalAngle()

        goal_angle = self.max_gapCenter
        #print("goal_angle: ", goal_angle*180/math.pi)
        #print("normalized_angle: ", self.normalizeAngle(self.car._angle)*180/math.pi)
        #print("car_angle: ", self.car._angle*180/math.pi)

        Alpha = 10      #tuning parameter 0,2,4,6,8,10. 0 means theta_final = theta goal
        Beta = 1
        if(self.max_gap > 20): #and self.car.lidar[90] <= 5):
            if(not self.obstacle_arry):
                return 0
            numerator = Alpha / min(self.obstacle_arry) * self.max_gapCenter + Beta*goal_angle
            denominator = Alpha / min(self.obstacle_arry) + Beta
        #print("nearest obstacle distance: " , min(self.obstacle_arry)) 
            theta_final = numerator / denominator - math.pi/2
        #print(theta_final) 

            return theta_final
        else:
            return 0

    def _followGap(self):
        self._findMaxGap()
        self._findMaxGapCenter()
        
        return(self._moveAwayFromWall(self.car.carLength-0.25,self._findHeading()))

                
    def avoidCollision(self):
        front_dist = self._getFrontDist()
        velocity = (front_dist - 2.5) / 7.5 * (self.car.speed_factor - 0.2) + 0.2
        if velocity < 0.35:
            velocity = 0.35
        elif velocity > 0.4:
            velocity = 0.4
        if front_dist < self.car.carLength*2:
            velocity = 0
        #print(velocity)
        self.car.changeMotorSpeed(velocity)

        buffer = math.pi/15
        new_angle = self._followGap()
        #print("avoid angle: "+str(new_angle))s

        diff = abs(new_angle - self.car.turnAngle)
        if(new_angle > self.car.turnAngle): # right
            # todo
            self.car.changeTurnAngle(self.car.turnAngle + diff)#weight*diff)
        elif(new_angle < self.car.turnAngle): # left
            self.car.changeTurnAngle(self.car.turnAngle - diff)#weight*diff)
    #-------------------------------------------------------------------------------------
            
    def main_funct(self):
        if(self.car.motorKill):
            self.car.changeMotorSpeed(0)
            self.car.changeTurnAngle(0)
	self.avoidCollision()
	self.publisher()

    #------------------------------------------------------------------------------------

    def update_lidar(self, data):
        #print("called")
        #print(data)
        # LIDAR data: (index, distance), where index goes from 1080 to 0
        # from the right going CCW, with 0 as center, 270deg = 0, -270deg = 1080
        # updates car's lidar array with the new tuple, converting to 0 to 180*n
        for i in range(180,len(data.ranges)-180):
            # these are values behind the LIDAR, we will discard them
            mapped_index = int(-(180.0*self.car.reading_number/720)*(i-900))
            self.car.lidar[mapped_index] = data.ranges[i]
        self.main_funct()   # lidar data updates very quickly, maybe trigger main_funct some other way

    def publisher(self):
        #todo, map vel and angle to [-100, 100]
        msg = drive_param()
        msg.velocity = self.car.motorSpeed
        print("[" + self.car.get_dur() + "] Motor: " + str(self.car.motorSpeed) + "%")
        msg.angle = 100*self.car.turnAngle/(math.pi/4) + 8

        self.pub.publish(msg)
        self.pub5.publish(msg.angle)

    def listener(self):
        rospy.Subscriber('scan', LaserScan, self.update_lidar) # calls driver main function when lidar is updated
        rospy.Subscriber('eBreak', Bool, self.kill_motors) # sets self.motorKill to true to stop motors for emergency

############################################################################

if __name__ == '__main__':
    rospy.init_node('driver', anonymous=True) # what is anonymous??
    car = Car()
    RacecarAI(car)
    rospy.spin()
