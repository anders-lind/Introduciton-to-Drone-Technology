import csv
import matplotlib.pyplot as plt
import math

# self.file.write("Time [s],easting [m],northing [m],altitude [m],local_rssi [dBm],remote_rssi[dBm]\n")

distances = []
easting = []
northing = []
local_rssi = []
remote_rssi = []

easting_start = 590723.742328429
northing_start = 6136521.76528065

first = True

data_file = '/home/anders/Documents/Drones_and_Autonomous_Systems/Introduction to Drone Technology/Lecture_08/data_premade.csv'



with open(data_file,'r') as csvfile: 
    data = csv.reader(csvfile, delimiter = ',')
    
      
    for row in data:
        if first:
            first = False
            continue

        easting.append(float(row[1]) - easting_start) 
        northing.append(float(row[2])  - northing_start) 
        local_rssi.append(float(row[4]))
        remote_rssi.append(float(row[5]))

        distances.append(math.sqrt( math.pow(easting[-1], 2) + math.pow(northing[-1], 2) ) )

plt.plot(distances, local_rssi, linestyle='--')
plt.plot(distances, remote_rssi, linestyle='-')
plt.show()