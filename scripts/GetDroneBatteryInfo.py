#!/usr/bin/env python

import rospy
from sensor_msgs.msg import BatteryState
import csv
import datetime

csv_filename = f'battery_data_{datetime.datetime.now():%Y-%m-%d_%H-%M-%S}.csv'

def battery_callback(data):
    with open(csv_filename, mode='a') as file:
        writer = csv.writer(file)
        
        if file.tell() == 0:  # Write header row if the file is empty
            field_names = [
                'Seq',
                'Stamp_secs',
                'Stamp_nsecs',
                'Frame_ID',
                'Voltage',
                'Temperature',
                'Current',
                'Charge',
                'Capacity',
                'Design_Capacity',
                'Percentage',
                'Power_Supply_Status',
                'Power_Supply_Health',
                'Power_Supply_Technology',
                'Present',
                'Cell_Voltage_1',
                'Cell_Voltage_2',
                'Cell_Voltage_3',
                'Cell_Voltage_4'
            ]
            writer.writerow(field_names)
        
        writer.writerow([
            data.header.seq,
            data.header.stamp.secs,
            data.header.stamp.nsecs,
            data.header.frame_id,
            data.voltage,
            data.temperature,
            data.current,
            data.charge,
            data.capacity,
            data.design_capacity,
            data.percentage,
            data.power_supply_status,
            data.power_supply_health,
            data.power_supply_technology,
            data.present,
            *data.cell_voltage
        ])

def status_timer_callback(event):
    rospy.loginfo("Program is running and receiving messages.")

if __name__ == '__main__':
    rospy.init_node('battery_subscriber', anonymous=True)
    
    # Set up the subscriber
    rospy.Subscriber('/mavros/battery', BatteryState, battery_callback)
    
    # Set up the status timer (5 seconds interval)
    status_timer = rospy.Timer(rospy.Duration(5.0), status_timer_callback)
    
    rospy.spin()  # Keeps the script running until the node is shut down
