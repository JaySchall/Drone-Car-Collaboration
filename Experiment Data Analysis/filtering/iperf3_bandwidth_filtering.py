import re
import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk, Button, filedialog, simpledialog

def read_iperf3_data(file_path):
    intervals = []
    transfers = []
    bitrates = []
    pattern = re.compile(r'(\d+\.\d+-\d+\.\d+)\s+sec\s+(\d+\.\d+)\s+MBytes\s+(\d+\.\d+)\s+Mbits/sec')

    with open(file_path, 'r') as file:
        for line in file:
            match = pattern.search(line)
            if match:
                interval, transfer, bitrate = match.groups()
                intervals.append(float(interval.split('-')[1]))
                transfers.append(float(transfer))
                bitrates.append(float(bitrate))

    df = pd.DataFrame({
        'Interval': intervals,
        'Transfer_MBytes': transfers,
        'Bitrate_Mbits_sec': bitrates
    })

    return df

def plot_iperf3_data(df, custom_title):
    plt.figure(figsize=(12, 8))
    plt.plot(df['Interval'], df['Transfer_MBytes'], label='Transfer (MBytes)', marker='o')
    plt.plot(df['Interval'], df['Bitrate_Mbits_sec'], label='Bitrate (Mbits/sec)', marker='x')
    plt.xlabel('Interval (sec)')
    plt.ylabel('Value')
    plt.title(f'Iperf3 Data: Interval vs Transfer and Bitrate {custom_title}')
    plt.legend()
    plt.show()

def select_file():
    file_path = filedialog.askopenfilename(filetypes=[('Text files', '*.txt')])
    if file_path:
        select_button.pack_forget()  # Hide the button after file is selected
        df = read_iperf3_data(file_path)
        custom_title = simpledialog.askstring("Input", "Enter custom name for graph title:")
        plot_iperf3_data(df, custom_title if custom_title else '')

if __name__ == "__main__":
    root = Tk()
    root.title('Iperf3 Data Plotter')
    root.geometry('300x100')
    select_button = Button(root, text="Press to select file", command=select_file)
    select_button.pack()
    print("Press the button to select the Iperf3 data file.")
    root.mainloop()
