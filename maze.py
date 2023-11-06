#!/usr/bin/env python

#Travel in GTG until an OA is triggered within d+e and d
import time
import math
import rospy
import numpy as np
import tf
import cv2
import csv

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from jairo_knn import crop_img, blob_center

########################################################################################################################################################
# START TRAINING
########################################################################################################################################################

train_year = '2019'
print('training...')
### Load training images and labels
with open('./'+ train_year +'imgs/label.txt', 'rb') as f: #train on all 2019
    reader = csv.reader(f)
    lines = list(reader)
num_train_imgs = len(lines)

###COLOR###
# this line reads in all images listed in the file in RGB, and resizes them to 33x25 pixels
train = np.array( [np.array(cv2.resize(crop_img(cv2.imread("./" + train_year + "imgs/"+lines[i][0]+".png",1)),(33,25))) for i in range(num_train_imgs)])

train_color = np.empty((num_train_imgs,25,33))
train_red = np.empty((num_train_imgs,25,33))
train_green = np.empty((num_train_imgs,25,33))
train_blue = np.empty((num_train_imgs,25,33))
for i in range(num_train_imgs):
        for j in range(25):
                for k in range(33):

                        #Feature Tables
                        train_color[i][j][k] = train[i][j][k].mean() #Mean of RGB
                        train_red[i][j][k] = train[i][j][k][0] #Red only
                        train_green[i][j][k] = train[i][j][k][1] #Green only
                        train_blue[i][j][k] = train[i][j][k][2] #Blue only

#Features
train_color_data = train_color.flatten().reshape(num_train_imgs, 33*25)
train_color_data = train_color_data.astype(np.float32)
train_red_data = train_red.flatten().reshape(num_train_imgs, 33*25)
train_red_data = train_red_data.astype(np.float32)
train_green_data = train_green.flatten().reshape(num_train_imgs, 33*25)
train_green_data = train_green_data.astype(np.float32)
train_blue_data = train_blue.flatten().reshape(num_train_imgs, 33*25)
train_blue_data = train_blue_data.astype(np.float32)
train_data_all = np.concatenate([train_color_data,train_blue_data,train_green_data,train_red_data])

#Labels
train_labels = np.array([np.int32(lines[i][1]) for i in range(num_train_imgs)])
train_labels_all = np.concatenate([train_labels,train_labels,train_labels,train_labels]) # len = number of feature vectors


#Train classifier
knn = cv2.ml.KNearest_create()
knn.train(train_data_all, cv2.ml.ROW_SAMPLE, train_labels_all)

print("done training")
########################################################################################################################################################
# END OF TRAINING
########################################################################################################################################################
classify = 0
rotating = 0
mode = 'none'
align = 0
desired_angle = 0
vel = Twist()
pos = Point()
object_pos = Point()
k_linear = .3 #.33
k_angular = .015 #0.06
d = .15
e = .10

def capture(sign):
	global frame
        np_arr = np.fromstring(sign.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def orientation(msg):
        pos.x = msg.x # x pos
        pos.y = msg.y # y pos
        pos.z = msg.z # Theta

def ao(obj):
	global classify
	global mode
	global rotating
	global desired_angle
	global align
	# Wall location based on lidar measurements and angles (0-359)
	object_pos.x = obj.x #distance to closest object (999 if nothing is near)
	object_pos.y = obj.y #angle to closest object
	front_dist = obj.z #distance right in from of robot

	# Start classifying
	if align == 1:
		rotating = 1 # rotate for alignment
		if classify == 1:
			align = 0
			classify = 0

			test_img = np.array(cv2.resize(cv2.cvtColor(crop_img(frame),cv2.COLOR_BGR2GRAY),(33,25)))
			test_img = test_img.flatten().reshape(1, 33*25)
			test_img = test_img.astype(np.float32)
			ret, results, neighbours, dist = knn.findNearest(test_img, 3)
			result = ret

			if result == 0.0: #wall
				mode = 'spin'
			elif result == 1.0: #left
				mode = 'left'
			elif result == 2.0: #right
				mode = 'right'
			elif result == 3.0 or result == 4.0: # stop
				mode = 'flip'
			else: # goal
				mode = 'done'
			print(mode)
			# Angle target
			if mode == 'left':
				if pos.z >= 0 and pos.z <= 269: # 0-269		
					desired_angle = pos.z + 90
				else: # 270-359
					desired_angle = pos.z + 90 - 360
			elif mode == 'right':
				if pos.z >= 90 and pos.z <= 359: # 90-359		
					desired_angle = pos.z - 90
				else: # 0-89
					desired_angle = pos.z - 90 + 360
			elif mode == 'flip':
				if pos.z >= 0 and pos.z <= 179: # 0-179		
					desired_angle = pos.z + 180
				else: # 180-359
					desired_angle = pos.z + 180 - 360
			elif mode == 'spin':
				desired_angle = pos.z
			else:
				desired_angle = pos.z
			#print('MODE:' + mode + ' pos: ' + str(pos.z) + 'desired_angle: ' + str(desired_angle))
			#print(mode + ' align=' + str(align) + ' classify=' + str(classify) + ' rotating=' + str(rotating))

		else:
			center = blob_center(frame) # center of blob. Left is positive. Right is positive
			angle_to_blob = -(center-240)*(62.2/480)
			desired_angle = pos.z + angle_to_blob
			if desired_angle < 0:
				desired_angle = desired_angle + 360
			
			#print('ALIGNING: pos: ' + str(pos.z) + 'desired_angle: ' + str(desired_angle))
			#print('ALIGNING align=' + str(align) + ' classify=' + str(classify) + ' rotating=' + str(rotating))
			align = 0
			classify = 1


	# Velocity Commands
	if rotating == 1: # Rotational velocity only


		# Anglular error while rotating
	        if (pos.z > desired_angle) and (pos.z-desired_angle > 180):
	                angle_error = 360 - (pos.z-desired_angle)
	        elif (pos.z > desired_angle) and (pos.z-desired_angle <= 180):
	                angle_error = -(pos.z-desired_angle)
	        elif (pos.z < desired_angle) and (desired_angle-pos.z <= 180):
	                angle_error = desired_angle-pos.z
	        elif (pos.z < desired_angle) and (desired_angle-pos.z > 180):
	                angle_error = -(360-(desired_angle-pos.z))
	        else:
	                angle_error = 0
		
		if angle_error<1 and angle_error>-1: # Successfully rotated
			if align == 0 and classify == 1: # aligned and ready to classify
				align = 1			
			elif align == 0 and classify == 0: # just classified
				rotating = 0
				#print('ALL ZEROS')
			v=0
			a=0
		else:
			v = 0
			a=angle_error*k_angular
		#print('pos: ' + str(pos.z) + ' desired: ' + str(desired_angle) + ' error: ' + str(angle_error))


	else: # Linear velocity only
		if front_dist <= 0.32 and front_dist >= 0.30: #stop
			v = 0
			a = 0
			align = 1 # align if close enough
		else: #continue straight
			if mode == 'done':
				v = 0
			else:
				v = .1

			if front_dist <= 0.6 and front_dist >= 0.4: #stop

				center = blob_center(frame) # center of blob. Left is positive. Right is positive
				angle_to_blob = -(center-240)*(62.2/480)
				desired_angle = pos.z + angle_to_blob
				if desired_angle < 0:
					desired_angle = desired_angle + 360

				# Anglular error while rotating
				if (pos.z > desired_angle) and (pos.z-desired_angle > 180):
					angle_error = 360 - (pos.z-desired_angle)
				elif (pos.z > desired_angle) and (pos.z-desired_angle <= 180):
					angle_error = -(pos.z-desired_angle)
				elif (pos.z < desired_angle) and (desired_angle-pos.z <= 180):
					angle_error = desired_angle-pos.z
				elif (pos.z < desired_angle) and (desired_angle-pos.z > 180):
					angle_error = -(360-(desired_angle-pos.z))
				else:
					angle_error = 0


				a=angle_error*0.04
			else:
				a = 0
			align = 0
			#print('straight' + ' align=' + str(align) + ' classify=' + str(classify) + ' rotating=' + str(rotating))


	vel.linear.x = v
        vel.angular.z = a
        pub1.publish(vel)


if __name__=='__main__':
        rospy.init_node('goToGoal')
        pub1 = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(10)

        rospy.Subscriber('/my_pos', Point, orientation) # x,y, theta
        rospy.Subscriber('/object_loc', Point, ao) # distance and agnle from closest object
	rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, capture) # raspicam image
        rate.sleep()
        rospy.spin()


##!/usr/bin/env python

##Travel in GTG until an OA is triggered within d+e and d
#import time
#import math
#import rospy
#import numpy as np
#import tf
#import cv2
#import csv

#from geometry_msgs.msg import Point, Twist
#from nav_msgs.msg import Odometry
#from sensor_msgs.msg import CompressedImage
#from jairo_knn import crop_img, blob_center

#########################################################################################################################################################
## START TRAINING
#########################################################################################################################################################

#train_year = '2019'
#print('training...')
#### Load training images and labels
#with open('./'+ train_year +'imgs/label.txt', 'rb') as f: #train on all 2019
#    reader = csv.reader(f)
#    lines = list(reader)
#num_train_imgs = len(lines)

####COLOR###
## this line reads in all images listed in the file in RGB, and resizes them to 33x25 pixels
#train = np.array( [np.array(cv2.resize(crop_img(cv2.imread("./" + train_year + "imgs/"+lines[i][0]+".png",1)),(33,25))) for i in range(num_train_imgs)])

#train_color = np.empty((num_train_imgs,25,33))
#train_red = np.empty((num_train_imgs,25,33))
#train_green = np.empty((num_train_imgs,25,33))
#train_blue = np.empty((num_train_imgs,25,33))
#for i in range(num_train_imgs):
#        for j in range(25):
#                for k in range(33):

#                        #Feature Tables
#                        train_color[i][j][k] = train[i][j][k].mean() #Mean of RGB
#                        train_red[i][j][k] = train[i][j][k][0] #Red only
#                        train_green[i][j][k] = train[i][j][k][1] #Green only
#                        train_blue[i][j][k] = train[i][j][k][2] #Blue only

##Features
#train_color_data = train_color.flatten().reshape(num_train_imgs, 33*25)
#train_color_data = train_color_data.astype(np.float32)
#train_red_data = train_red.flatten().reshape(num_train_imgs, 33*25)
#train_red_data = train_red_data.astype(np.float32)
#train_green_data = train_green.flatten().reshape(num_train_imgs, 33*25)
#train_green_data = train_green_data.astype(np.float32)
#train_blue_data = train_blue.flatten().reshape(num_train_imgs, 33*25)
#train_blue_data = train_blue_data.astype(np.float32)
#train_data_all = np.concatenate([train_color_data,train_blue_data,train_green_data,train_red_data])

##Labels
#train_labels = np.array([np.int32(lines[i][1]) for i in range(num_train_imgs)])
#train_labels_all = np.concatenate([train_labels,train_labels,train_labels,train_labels]) # len = number of feature vectors


##Train classifier
#knn = cv2.ml.KNearest_create()
#knn.train(train_data_all, cv2.ml.ROW_SAMPLE, train_labels_all)

#print("done training")
#########################################################################################################################################################
## END OF TRAINING
#########################################################################################################################################################
#classify = 0
#rotating = 0
#mode = 'none'
#desired_angle = 0
#vel = Twist()
#pos = Point()
#object_pos = Point()
#k_linear = .3 #.33
#k_angular = .02 #0.06
#d = .15
#e = .10

#def capture(sign):
#	global frame
#        np_arr = np.fromstring(sign.data, np.uint8)
#        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#def orientation(msg):
#        pos.x = msg.x # x pos
#        pos.y = msg.y # y pos
#        pos.z = msg.z # Theta

#def ao(obj):
#	global classify
#	global mode
#	global rotating
#	global desired_angle

#	# Wall location based on lidar measurements and angles (0-359)
#	object_pos.x = obj.x #distance to closest object (999 if nothing is near)
#	object_pos.y = obj.y #angle to closest object
#	front_dist = obj.z #distance right in from of robot

#	# Start classifying
#	if classify == 1:
#		classify = 0 # reset flag
#		rotating = 1 # wait for robot to stop rotating
#		test_img = np.array(cv2.resize(cv2.cvtColor(crop_img(frame),cv2.COLOR_BGR2GRAY),(33,25)))
#		test_img = test_img.flatten().reshape(1, 33*25)
#		test_img = test_img.astype(np.float32)
#        	ret, results, neighbours, dist = knn.findNearest(test_img, 3)
#		result = ret

#		if result == 0.0: #wall
#			mode = 'spin'
#		elif result == 1.0: #left
#			mode = 'left'
#		elif result == 2.0: #right
#			mode = 'right'
#		elif result == 3.0 or result == 4.0: # stop
#			mode = 'flip'
#		else: # goal
#			mode = 'done'

#		# Angle target
#		if mode == 'left':
#			if pos.z >= 0 and pos.z <= 269: # 0-269		
#				desired_angle = pos.z + 90
#			else: # 270-359
#				desired_angle = pos.z + 90 - 360
#		elif mode == 'right':
#			if pos.z >= 90 and pos.z <= 359: # 90-359		
#				desired_angle = pos.z - 90
#			else: # 0-89
#				desired_angle = pos.z - 90 + 360
#		elif mode == 'flip':
#			if pos.z >= 0 and pos.z <= 179: # 0-179		
#				desired_angle = pos.z + 180
#			else: # 180-359
#				desired_angle = pos.z + 180 - 360
#		elif mode == 'spin':
#			desired_angle = pos.z
#		else:
#			desired_angle = pos.z

#		print(mode)

#	# Velocity Commands
#	if rotating == 1: # Rotational velocity only


#		# Anglular error while rotating
#	        if (pos.z > desired_angle) and (pos.z-desired_angle > 180):
#	                angle_error = 360 - (pos.z-desired_angle)
#	        elif (pos.z > desired_angle) and (pos.z-desired_angle <= 180):
#	                angle_error = -(pos.z-desired_angle)
#	        elif (pos.z < desired_angle) and (desired_angle-pos.z <= 180):
#	                angle_error = desired_angle-pos.z
#	        elif (pos.z < desired_angle) and (desired_angle-pos.z > 180):
#	                angle_error = -(360-(desired_angle-pos.z))
#	        else:
#	                angle_error = 0
#		
#		if angle_error<0.5 and angle_error>-0.5: # Successfully rotated
#			v=0
#			a=0
#			rotating = 0
#		else:
#			v = 0
#			a=angle_error*k_angular
#		print('pos: ' + str(pos.z) + ' desired: ' + str(desired_angle) + ' error: ' + str(angle_error))


#	else: # Linear velocity only
#		if front_dist <= 0.36 and front_dist >= 0.30: #stop
#			v = 0
#			a = 0
#			classify = 1
#		else: #continue straight
#			v = .1
#			a = 0
#			classify = 0


#	vel.linear.x = v
#        vel.angular.z = a
#        pub1.publish(vel)


#if __name__=='__main__':
#        rospy.init_node('goToGoal')
#        pub1 = rospy.Publisher('cmd_vel', Twist, queue_size=1)
#        rate = rospy.Rate(10)

#        rospy.Subscriber('/my_pos', Point, orientation) # x,y, theta
#        rospy.Subscriber('/object_loc', Point, ao) # distance and agnle from closest object
#	rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, capture) # raspicam image
#        rate.sleep()
#        rospy.spin()


