import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter

#part 1: filter

# Define the path to the input CSV file
csv_file_path = r"C:\Users\demet\Documents\GitHub\Drone-Car-Collaboration\Experiment Data Analysis\Experiment 1 - load balance - 7 objects - red-yellow-blue\run 1\battery_data_2023-07-23_12-41-52.csv"


# Read the CSV file into a pandas DataFrame
df = pd.read_csv(csv_file_path)

# Filter and calculate wattage (voltage * absolute(current))
filtered_df = df[['Seq', 'Stamp_secs', 'Stamp_nsecs', 'Voltage', 'Current']]
filtered_df['Current'] = filtered_df['Current'].abs()  # Take the absolute value of current
filtered_df['Wattage'] = filtered_df['Voltage'] * filtered_df['Current']

# Calculate the initial timestamp value
initial_timestamp = filtered_df.loc[0, 'Stamp_secs'] + filtered_df.loc[0, 'Stamp_nsecs'] * 1e-9

# Adjust timestamps to start from 0 seconds
filtered_df['Timestamp'] = filtered_df.apply(lambda row: (row['Stamp_secs'] + row['Stamp_nsecs'] * 1e-9) - initial_timestamp, axis=1)

# Define the path to the output CSV file
output_csv_file_path =r"C:\Users\demet\Documents\GitHub\Drone-Car-Collaboration\Experiment Data Analysis\Experiment 1 - load balance - 7 objects - red-yellow-blue\run 1\battery_data_2023-07-23_12-41-52-FILTERED.csv"

# Save the filtered data with adjusted timestamps and wattage to the output CSV file
adjusted_filtered_df = filtered_df[['Seq', 'Timestamp', 'Wattage']]  # Select relevant columns
adjusted_filtered_df.to_csv(output_csv_file_path, index=False)

print("Filtered data with adjusted timestamps and wattage has been saved to:", output_csv_file_path)

#part 2: generate graphics

#wattage v time:

# Read the filtered CSV file into a pandas DataFrame
adjusted_filtered_df = pd.read_csv(output_csv_file_path)

# Plot wattage versus time
plt.figure(figsize=(10, 6))
plt.plot(adjusted_filtered_df['Timestamp'], adjusted_filtered_df['Wattage'], marker='o', linestyle='-', color='b')
plt.xlabel('Time (seconds)')
plt.ylabel('Wattage')
plt.title('Wattage vs Time - Experiment 1, Run 1 - Load Balance Drone-Car')
plt.grid()

# Format x and y tick labels with 4 decimal places
decimal_formatter = FormatStrFormatter('%0.4f')
plt.gca().xaxis.set_major_formatter(decimal_formatter)
plt.gca().yaxis.set_major_formatter(decimal_formatter)

plt.show()

#avg wattage every 5 seconds v time:

# Read the filtered CSV file into a pandas DataFrame
adjusted_filtered_df = pd.read_csv(output_csv_file_path)

# Convert the 'Timestamp' column to a datetime object for resampling
adjusted_filtered_df['Timestamp'] = pd.to_datetime(adjusted_filtered_df['Timestamp'], unit='s')

# Resample the data into 5-second intervals and calculate the mean wattage
resampled_df = adjusted_filtered_df.resample('5S', on='Timestamp').mean()

# Plot average wattage versus time
plt.figure(figsize=(10, 6))
plt.plot(resampled_df.index, resampled_df['Wattage'], marker='o', linestyle='-', color='b')
plt.xlabel('Time')
plt.ylabel('Average Wattage')
plt.title('Average Wattage vs Time (5-second intervals) - Experiment 1, Run 1 - Load Balance Drone-Car')
plt.grid()

# Format x tick labels with date and time format
plt.gca().xaxis.set_major_formatter(plt.matplotlib.dates.DateFormatter('%H:%M:%S'))

plt.show()