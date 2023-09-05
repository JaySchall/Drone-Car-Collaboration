
import subprocess
import time
import os
import signal

def run_scripts():
    # Initialize empty list to store PIDs
    pids = []

    try:
        # Run GetDroneBatteryInfo.py
        print("Starting GetDroneBatteryInfo.py...")
        drone_battery_process = subprocess.Popen(["python3", "GetDroneBatteryInfo.py"])
        pids.append(drone_battery_process.pid)

        # Run iperf3_server_script.sh
        print("Starting iperf3_server_script.sh...")
        iperf3_process = subprocess.Popen(["bash", "iperf3_server_script.sh"])
        pids.append(iperf3_process.pid)

        # Run ping_rtt_script.sh
        print("Starting ping_rtt_script.sh...")
        ping_rtt_process = subprocess.Popen(["bash", "ping_rtt_script.sh"])
        pids.append(ping_rtt_process.pid)

        print(f"Running scripts with PIDs: {pids}")

        # Keep the script running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt. Terminating all scripts.")
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except Exception as e:
                print(f"Failed to terminate process with PID {pid}: {e}")
        print("All scripts terminated. Exiting.")

if __name__ == "__main__":
    run_scripts()
