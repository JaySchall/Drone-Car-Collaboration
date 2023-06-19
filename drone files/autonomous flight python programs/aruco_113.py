# Information: https://clover.coex.tech/programming

import rospy
from clover import srv
from std_srvs.srv import Trigger

def timer(seconds):
    print(f"Waiting for {seconds} seconds...")
    for i in range(seconds):
        rospy.sleep(1)
        print(f"{seconds-i} seconds remaining...")
    print("Launching program...\n***FLIGHT INITIATED!***")

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

#note: default speed is 0.5 m/s
#note: only the first call to navigate() has its auto_arm=True
    #auto_arm param arms the drone and transitions it to OFFBOARD flight mode

#INITIATE FLIGHT COUNTDOWN TIMER:
timer(10)

#BEGIN FLIGHT:

#always make drone take off relative to its own body frame
print('Take off and hover 1 m above body frame')
navigate(x=0, y=0, z=1, speed=0.2, frame_id='body', auto_arm=True)

#wait for 5 seconds
rospy.sleep(5)

print('Hover 1 m above aruco marker 113')
navigate(x=0, y=0, z=1, speed=0.2, frame_id='aruco_113')

# Wait for x seconds
print('Hovering for 15 seconds...')
rospy.sleep(15)

#move drone in x direction 1.5m (x direction points +dir to right of marker)
print('Hover 1 m above and 1.5 meters to the right of aruco marker 113')
navigate(x=1.5, y=0, z=1, speed=0.2, frame_id='aruco_113')

#wait for 15 seconds
print('Hovering for 15 seconds...')
rospy.sleep(15)

#First, decrease drone height, then
#land the drone --> using drone body frame of reference
print('Performing Landing...')
navigate(x=0, y=0, z=0.2, speed=0.1, frame_id='body')
rospy.sleep(10)
land()




#other notes on body frame: The body frame of reference is typically
#defined using a set of three orthogonal axes (x, y, and z)
#that are fixed to the drone's frame, with the origin located 
#at the center of mass of the drone. 
#The x-axis is typically aligned with the drone's forward direction, 
#the y-axis with the rightward direction,
#and the z-axis with the downward direction.
