import pandas as pd
import matplotlib.pyplot as plt

# Read data from the CSV file
data = pd.read_csv('./data/throughput_latency.csv', header=None, names=['x', 'throughput', 'latency'])

# Group the data by the x-axis values and calculate the mean of the y-axis values for each group
grouped_data = data.groupby('x').mean().reset_index()

# Create the throughput plot
plt.figure(figsize=(8, 6))
plt.scatter(data['x'], data['throughput'], s=20, alpha=0.5, label='Individual data points', marker='o')
plt.plot(grouped_data['x'], grouped_data['throughput'], color='red', label='Mean values', linewidth=2)
plt.xlabel('Payload Size (Bytes)', fontsize=14) # increased fontsize to 14
plt.ylabel('Throughput (KB/s)', fontsize=14) # increased fontsize to 14
plt.title('Throughput vs Payload Size', fontsize=16) # increased fontsize to 16
plt.legend(fontsize=10)
plt.xscale('log', base=2)
plt.xticks(data['x'].unique())
plt.grid(axis='x')
plt.grid(axis='y')

# Create the latency plot
plt.figure(figsize=(8, 6))
plt.scatter(data['x'], data['latency'], s=20, alpha=0.5, label='Individual data points', marker='o')
plt.plot(grouped_data['x'], grouped_data['latency'], color='red', label='Mean values', linewidth=2)
plt.xlabel('Payload Size (Bytes)', fontsize=14) # increased fontsize to 14
plt.ylabel('Latency (ms)', fontsize=14) # increased fontsize to 14
plt.title('Latency vs Payload Size', fontsize=16) # increased fontsize to 16
plt.legend(fontsize=10)
plt.xscale('log', base=2)
plt.xticks(data['x'].unique())
plt.grid(axis='x')
plt.grid(axis='y')

# Show both plots
plt.show()
