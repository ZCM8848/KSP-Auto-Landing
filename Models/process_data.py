import csv
import matplotlib.pyplot as plt
import numpy as np

density = []
altitude = []
air_speed = []

with open(r"D:\Python_projects\KSP-Auto-Landing\Models\stock\raw_data\stock.csv","r") as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        altitude.append(float(line[0]))
        density.append(float(line[1]))
        air_speed.append(float(line[2]))

    z1 = np.polyfit(altitude, density, 5)
    print(z1)