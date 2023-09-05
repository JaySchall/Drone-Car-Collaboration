import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import os
from tkinter import filedialog, simpledialog, Tk

# Part 1: Filter

# Open a dialog box to ask the user for the CSV file path
root = Tk()
root.withdraw()  # Hide the Tkinter root window
csv_file_path = filedialog.askopenfilename(title="Select CSV file", filetypes=[("CSV files", "*.csv")])

# Check if a file was selected
if not csv_file_path:
    print("No file selected. Exiting.")
    exit()

# Read the CSV file into a pandas DataFrame
df = pd.read_csv(csv_file_path)

# Filter and calculate wattage (voltage * absolute(current))
filtered_df = df[['Seq', 'Stamp_secs', 'Stamp_nsecs', 'Voltage', 'Current']].copy()
filtered_df['Current'] = filtered_df['Current'].abs()
filtered_df['Wattage'] = filtered_df['Voltage'] * filtered_df['Current']

# Calculate the initial timestamp value
initial_timestamp = filtered_df.loc[0, 'Stamp_secs'] + filtered_df.loc[0, 'Stamp_nsecs'] * 1e-9

# Adjust timestamps to start from 0 seconds
filtered_df['Timestamp'] = filtered_df.apply(lambda row: (row['Stamp_secs'] + row['Stamp_nsecs'] * 1e-9) - initial_timestamp, axis=1)

# Define the path to the output CSV file in the same directory as the input file
output_csv_file_path = os.path.join(os.path.dirname(csv_file_path), "filtered_data_with_wattage.csv")

# Save the filtered data with adjusted timestamps and wattage to the output CSV file
adjusted_filtered_df = filtered_df[['Seq', 'Timestamp', 'Wattage']]
adjusted_filtered_df.to_csv(output_csv_file_path, index=False)

print("Filtered data with adjusted timestamps and wattage has been saved to:", output_csv_file_path)

# Ask the user for a custom title to append to the graph title
custom_title = simpledialog.askstring("Input", "Enter a custom title to append to the graph title:")

# Part 2: Generate Graphics

# Wattage vs time:

# Read the filtered CSV file into a pandas DataFrame
adjusted_filtered_df = pd.read_csv(output_csv_file_path)

# Plot wattage versus time
plt.figure(figsize=(10, 6))
default_title = 'Wattage vs Time'
plt.title(f"{default_title} {custom_title if custom_title else ''}")
plt.plot(adjusted_filtered_df['Timestamp'], adjusted_filtered_df['Wattage'], marker='o', linestyle='-', color='b')
plt.xlabel('Time (seconds)')
plt.ylabel('Wattage')
plt.grid()

# Format x and y tick labels with 4 decimal places
decimal_formatter = FormatStrFormatter('%0.4f')
plt.gca().xaxis.set_major_formatter(decimal_formatter)
plt.gca().yaxis.set_major_formatter(decimal_formatter)

plt.show()

# Avg wattage every 5 seconds vs time:

# Read the filtered CSV file into a pandas DataFrame
adjusted_filtered_df = pd.read_csv(output_csv_file_path)

# Convert the 'Timestamp' column to a datetime object for resampling
adjusted_filtered_df['Timestamp'] = pd.to_datetime(adjusted_filtered_df['Timestamp'], unit='s')

# Resample the data into 5-second intervals and calculate the mean wattage
resampled_df = adjusted_filtered_df.resample('5S', on='Timestamp').mean()

# Plot average wattage versus time
plt.figure(figsize=(10, 6))
default_title = 'Average Wattage vs Time (5-second intervals)'
plt.title(f"{default_title} {custom_title if custom_title else ''}")
plt.plot(resampled_df.index, resampled_df['Wattage'], marker='o', linestyle='-', color='b')
plt.xlabel('Time')
plt.ylabel('Average Wattage')
plt.grid()

# Format x tick labels with date and time format
plt.gca().xaxis.set_major_formatter(plt.matplotlib.dates.DateFormatter('%H:%M:%S'))

plt.show()
