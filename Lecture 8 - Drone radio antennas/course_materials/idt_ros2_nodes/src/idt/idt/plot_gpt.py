import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load the CSV files
file_homemade = '/home/anders/Documents/Drones_and_Autonomous_Systems/Introduction to Drone Technology/Lecture_08/data_homemade.csv'
file_premade = '/home/anders/Documents/Drones_and_Autonomous_Systems/Introduction to Drone Technology/Lecture_08/data_premade.csv'

# Read the data into dataframes
data_homemade = pd.read_csv(file_homemade)
data_premade = pd.read_csv(file_premade)

# Extract relevant columns from both datasets
homemade_data = data_homemade[["easting [m]", "local_rssi [dBm]", "remote_rssi[dBm]"]]
premade_data = data_premade[["easting [m]", "local_rssi [dBm]", "remote_rssi[dBm]"]]

# Calculate the starting easting and northing for each dataset
homemade_start = homemade_data["easting [m]"].iloc[0], data_homemade["northing [m]"].iloc[0]
premade_start = premade_data["easting [m]"].iloc[0], data_premade["northing [m]"].iloc[0]

# Calculate Euclidean distances
homemade_data["distance (m)"] = np.sqrt(
    (homemade_data["easting [m]"] - homemade_start[0]) ** 2 +
    (data_homemade["northing [m]"] - homemade_start[1]) ** 2
)
premade_data["distance (m)"] = np.sqrt(
    (premade_data["easting [m]"] - premade_start[0]) ** 2 +
    (data_premade["northing [m]"] - premade_start[1]) ** 2
)

# Plot RSSI as a function of Euclidean distance
plt.figure(figsize=(14, 8))

# Homemade antenna
plt.plot(np.array(homemade_data["distance (m)"]), np.array(homemade_data["local_rssi [dBm]"]), 
         label="Homemade Local RSSI", linestyle='--', color='blue')
plt.plot(np.array(homemade_data["distance (m)"]), np.array(homemade_data["remote_rssi[dBm]"]), 
         label="Homemade Remote RSSI", linestyle='-', color='blue')

# Premade antenna
plt.plot(np.array(premade_data["distance (m)"]), np.array(premade_data["local_rssi [dBm]"]), 
         label="Premade Local RSSI", linestyle='--', color='orange')
plt.plot(np.array(premade_data["distance (m)"]), np.array(premade_data["remote_rssi[dBm]"]), 
         label="Premade Remote RSSI", linestyle='-', color='orange')

# Adding titles and labels
plt.title("RSSI as a Function of Euclidean Distance", fontsize=16)
plt.xlabel("Distance (m)", fontsize=14)
plt.ylabel("RSSI (dBm)", fontsize=14)
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(fontsize=12)
plt.show()