import init_library as robot  #Library tha handles all the serial commands to arduino AtMega
import time
import numpy as np


robot.enablePID(1) #Enables PI (Proportional Integral) on robot this causes two things, we no longer can send regular robot.motors() commands because PI will immedietly 0 them. It allows us to use motorsRPM command which is regulated by our PI loop


robot.motorsRPM(0,0,0) #This takes a rpm value for example robots.motorsRPM(180,0,0) would make wheel one rotate 180 times a min. I'd leave this here as it makes sure your motors are off at start and this can be helpful (max right now is around 180RPM)


#Kinematic equation to give velocities in terms of robots x,y,theta direction. x = m/s y = m/s theta = radians/s
#this is what you implemented last week
def move(xd,yd,thetad):

	r = 0.03 # radius of each wheel [m]
	l = 0.19 # distance from each wheel to the point of reference [m]
 
	xd_des = xd # velocity in the x-direction in the local frame [m/s]
	yd_des = yd # velocity in the y-direction in the local frame [m/s]
	thd_des = thetad # velocity in the x-direction in the local frame [rad/sa]
 
	vel_des = np.array([xd_des,yd_des,thd_des])[:,None]
 
	FK_M = (2*np.pi*r/60)*np.array([1/np.sqrt(3),0,-1/np.sqrt(3),-1/3,2/3,-1/3,-1/(3*l),-1/(3*l),-1/(3*l)]).reshape(3,3) # Forward kinematics matrix
 
	IK_M = np.linalg.inv(FK_M) # Inverse kinematics matrix
 
	motor_spd_vec = np.dot(IK_M,vel_des)
 
	wheel1RPM = motor_spd_vec[0] # motor 2 speed [rpm]
	wheel0RPM = motor_spd_vec[1] # motor 1 speed [rpm]
	wheel2RPM = motor_spd_vec[2] # motor 3 speed [rpm]


	print("Wheel0 RPM: " +str(wheel0RPM))
	print("Wheel1 RPM: " +str(wheel1RPM))
	print("Wheel2 RPM: " +str(wheel2RPM))


	robot.motorsRPM(int(wheel0RPM),int(wheel1RPM),int(wheel2RPM))




#this first while loop will reset after the time you enter elapses. 
while True:
		timer = input("Enter time to run: ")
		x = float(input("enter desired x velocity (max .5 m/s): "))
		y = float(input("enter desired y velocity (max .5 m/s): "))
		theta = float(input("enter desired angular velocity (max π rad/s): "))
		start = time.time()
		move(x,y,theta)
		
		#set your old encoder values
		oldEncoder0 = robot.encoder(0)
		oldEncoder1 = robot.encoder(1)
		oldEncoder2 = robot.encoder(2)
		
		#reset odemetry to an origin of xy(0,0) rotation(0)
		old_x = 0
		old_y = 0
		old_t = 0
		
		#your second loop will be for calculating odemetry for the time period you entered above
		while True:
			
			
			#You will enter your code here to calculate odemetry remember its building up over time (integral)
			
			
			
			#this will end the loop after the time has passed
			if time.time()-float(start) >= float(timer):
				move(0,0,0)
				break
