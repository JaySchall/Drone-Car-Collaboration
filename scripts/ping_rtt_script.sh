#!/bin/bash

# Define the target address and create the output file name with the current date
target_address="192.168.11.133"  # Change this to the desired IP address
current_date=$(date +"%Y-%m-%d_%H-%M-%S")
output_file="ping_results_${current_date}.txt"

# Function to perform the continuous pinging
perform_pinging() {
    while :; do
        ping -i 1 -c 1 "$target_address" >> "$output_file"
    done
}

# Function to gracefully stop the script
cleanup() {
    echo "Stopping the ping test..."
    kill "$ping_pid"  # Stop the background pinging process
    echo "Ping test stopped."
    exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM signals to run the cleanup function
trap cleanup SIGINT SIGTERM

# Start pinging in the background and capture the PID
perform_pinging &
ping_pid=$!

echo "Ping test is running. Results are being written to $output_file."

# Wait for user to stop the script
read -rp "Press Enter to stop the ping test..."

# Run the cleanup function manually if the user didn't use Ctrl+C
cleanup