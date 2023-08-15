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

# Get number of oscillations from user, and also hover time and offset
oscillations = int(input('Enter the number of oscillations: '))
hover_time = int(input('Enter the hover time to be used: '))
x_aruco_offset = float(input('Enter the x offset to be used: '))
if oscillations <= 0 or hover_time < 5 or x_aruco_offset < 0.5:
    print('number of oscillations <= 0, exiting program...')
    sys.exit(0)

#START FLIGHT COUNTDOWN TIMER:
timer(10)

#BEGIN FLIGHT:

# Always make drone take off relative to its own body frame
print('Take off and hover 1 m above body frame')
navigate(x=0, y=0, z=1.5, speed=0.2, frame_id='body', auto_arm=True)

# Wait for 5 seconds
rospy.sleep(5)

for i in range(oscillations):
    print(f'Performing oscillation {i + 1}...')

    # Hover 1 m above aruco marker 113
    print('Hover 1 m above aruco marker 113')
    navigate(x=0, y=0, z=1.5, speed=0.2, frame_id='aruco_113')

    # Wait for 10 seconds
    print(f'Hovering for {hover_time} seconds...')
    rospy.sleep(10)

    # Move drone in x direction (x direction points +dir to right of marker)
        #when drone is facing in +y direction relative to aruco marker coordinates.
    print(f'Hover 1 m above, {x_aruco_offset} m right (x offset) of aruco marker 113')
    navigate(x=x_aruco_offset, y=0, z=1.5, speed=0.2, frame_id='aruco_113')

    # Wait for 10 seconds
    print(f'Hovering for {hover_time} seconds...')
    rospy.sleep(10)

# First, decrease drone height, then land the drone using drone body frame of reference
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
