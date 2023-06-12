import matplotlib.pyplot as plt
import pandas as pd

surface_data = pd.read_excel('Sidewalk_adxl-3200Hz-16g_imu-562Hz-16g.xlsx')

# Extract the x and y coordinates from the data
x = surface_data['Samples'].tolist()
y1 = surface_data['ADXL Magnitude'].tolist()
y2 = surface_data['IMU Magnitude'].tolist()

# Create a figure and axis object
fig, ax = plt.subplots()

# Plot the line
ax.plot(x, y1, label ='ADXL Magnitude')
ax.plot(x, y2, label = 'IMU Magnitude')

# Add labels and title
ax.set_xlabel('Samples')
ax.set_ylabel('Magnitude')
ax.legend()
ax.set_title('Surface acceleration data')

fig.tight_layout()
# Display the plot
plt.show()
