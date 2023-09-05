#!/bin/bash

# Define the target address and create the output file name with the current date
target_address="192.168.11.133"  # Change this to the desired IP address
current_date=$(date +"%Y-%m-%d_%H-%M-%S")
output_file="ping_results_${current_date}.txt"

# Function to perform the continuous pinging
perform_pinging() {
    while true; do
        ping -i 1 -c 1 $target_address >> $output_file
    done
}

# Start pinging in the background
perform_pinging &

echo "Ping test is running. Results are being written to $output_file."

# Wait for user to stop the script
read -p "Press Enter to stop the ping test..."

# Stop the background pinging process
killall ping

echo "Ping test stopped."
