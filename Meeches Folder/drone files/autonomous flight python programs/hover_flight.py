# Information: https://clover.coex.tech/programming
import sys
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

hover_time= int(input('input select a hover time >= 5 seconds: '))
if hover_time < 5:
    print('invalid hover time...exiting program...')
    sys.exit(0)

#INITIATE FLIGHT COUNTDOWN TIMER:
timer(35)

#BEGIN FLIGHT:

print(f'Take off and hover 1 m above the ground for {hover_time} seconds...')
navigate(x=0, y=0, z=1, speed=0.2, frame_id='body', auto_arm=True)

# Wait for x seconds
rospy.sleep(hover_time)

#print('Fly forward 1 m')
#navigate(x=1, y=0, z=0, frame_id='body')

# Wait for 5 seconds
#rospy.sleep(5)

print('Prepare for landing')
# initiate landing
print('drone is landing...')
land()
