import re
import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk
from tkinter import filedialog, simpledialog

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

def open_file_dialog():
    root = tk.Tk()
    root.withdraw()  # Hide the main window
    
    file_path = filedialog.askopenfilename(title="Select RTT Data File")
    return file_path

def get_custom_title(default_title):
    root = tk.Tk()
    root.withdraw()  # Hide the main window
    
    custom_title = simpledialog.askstring("Custom Title", "Enter additional text for the graph title:")
    if custom_title:
        return f"{default_title} - {custom_title}"
    else:
        return default_title

# Create a simple GUI window
gui = tk.Tk()
gui.title("RTT Data Analysis")

def browse_file():
    file_path = open_file_dialog()
    if file_path:
        gui.withdraw()  # Hide the GUI window
        custom_title = get_custom_title("Aggregated RTT Data Analysis")

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
        plt.title(custom_title)
        plt.legend()
        plt.grid(True)
        plt.show()
        gui.quit()

# Display a message
message_label = tk.Label(gui, text="Please select an RTT data file:")
message_label.pack(pady=10)

# Button to trigger file dialog
browse_button = tk.Button(gui, text="Browse", command=browse_file)
browse_button.pack()

# Start the GUI event loop
gui.mainloop()
