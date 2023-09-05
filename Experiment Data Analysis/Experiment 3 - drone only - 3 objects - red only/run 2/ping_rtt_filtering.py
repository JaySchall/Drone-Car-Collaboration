
import re
import matplotlib.pyplot as plt
import numpy as np

def aggregate_data(data, N):
    """Aggregate data by taking the average of every N data points, padding with zeros if necessary."""
    remainder = len(data) % N
    if remainder != 0:
        pad_size = N - remainder
        data = np.pad(data, (0, pad_size), 'constant')
    return np.mean(np.array(data).reshape(-1, N), axis=1)

# Function to read RTT data from a file and return lists of min, avg, and max RTT values
def read_rtt_data(file_path):
    rtt_min_values = []
    rtt_avg_values = []
    rtt_max_values = []
    
    rtt_pattern_all = re.compile(r'rtt min/avg/max/mdev = ([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+) ms')
    
    with open(file_path, 'r') as file:
        for line in file:
            match = rtt_pattern_all.search(line)
            if match:
                rtt_min = float(match.group(1))
                rtt_avg = float(match.group(2))
                rtt_max = float(match.group(3))
                
                rtt_min_values.append(rtt_min)
                rtt_avg_values.append(rtt_avg)
                rtt_max_values.append(rtt_max)
                
    return rtt_min_values, rtt_avg_values, rtt_max_values

# File path (Please change this to the actual location of your data file)
file_path = "C:\\Users\\demet\\Documents\\GitHub\\Drone-Car-Collaboration\\Experiment Data Analysis\\filtering\\ping_rtt_data.txt"

# Read RTT data
rtt_min, rtt_avg, rtt_max = read_rtt_data(file_path)

# Aggregate the data (N=10)
N = 10
agg_rtt_min = aggregate_data(rtt_min, N)
agg_rtt_avg = aggregate_data(rtt_avg, N)
agg_rtt_max = aggregate_data(rtt_max, N)

# Generate the line plot
plt.figure(figsize=(12, 6))
#plt.plot(agg_rtt_min, label='Min RTT (Aggregated)', linestyle='--')
plt.plot(agg_rtt_avg, label='Avg RTT (Aggregated)')
#plt.plot(agg_rtt_max, label='Max RTT (Aggregated)', linestyle='-.')

plt.xlabel('Ping Index (Aggregated)')
plt.ylabel('RTT (ms)')
plt.title('Aggregated RTT Data Analysis')
plt.legend()
plt.grid(True)
plt.show()
