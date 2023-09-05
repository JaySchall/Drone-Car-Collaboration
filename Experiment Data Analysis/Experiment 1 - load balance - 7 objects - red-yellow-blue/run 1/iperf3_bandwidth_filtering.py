
import re
import pandas as pd
import matplotlib.pyplot as plt

def read_iperf3_data(file_path):
    # Initialize empty lists to store the extracted data
    intervals = []
    transfers = []
    bitrates = []

    # Regular expression pattern to extract relevant data
    pattern = re.compile(r'(\d+\.\d+-\d+\.\d+)\s+sec\s+(\d+\.\d+)\s+MBytes\s+(\d+\.\d+)\s+Mbits/sec')

    # Open file and read line by line
    with open(file_path, 'r') as file:
        for line in file:
            match = pattern.search(line)
            if match:
                # Extract and store the data
                interval, transfer, bitrate = match.groups()
                intervals.append(float(interval.split('-')[1]))  # Store the end of the interval as a float
                transfers.append(float(transfer))
                bitrates.append(float(bitrate))

    # Create a DataFrame from the extracted data
    df = pd.DataFrame({
        'Interval': intervals,
        'Transfer_MBytes': transfers,
        'Bitrate_Mbits_sec': bitrates
    })

    return df

def plot_iperf3_data(df):
    # Plot the data
    plt.figure(figsize=(12, 8))

    # Plot Interval, Transfer, and Bitrate on the same graph
    plt.plot(df['Interval'], df['Transfer_MBytes'], label='Transfer (MBytes)', marker='o')
    plt.plot(df['Interval'], df['Bitrate_Mbits_sec'], label='Bitrate (Mbits/sec)', marker='x')

    # Add labels and title
    plt.xlabel('Interval (sec)')
    plt.ylabel('Value')
    plt.title('Iperf3 Data: Interval vs Transfer and Bitrate - Experiment 1, Run 1 - Load Balance Drone-Car')

    # Add a legend
    plt.legend()

    # Show the plot
    plt.show()

if __name__ == "__main__":
    file_path = r"C:\Users\demet\Documents\GitHub\Drone-Car-Collaboration\Experiment Data Analysis\Experiment 1 - load balance - 7 objects - red-yellow-blue\run 1\iperf3_TCP_Test_server.txt"
    df = read_iperf3_data(file_path)
    plot_iperf3_data(df)
