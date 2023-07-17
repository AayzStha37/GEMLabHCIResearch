import matplotlib.pyplot as plt
import pandas as pd
cnt=1
surface_data = pd.read_excel('Sidewalk_adxl-3200Hz-16g_imu-562Hz-16g.xlsx',sheet_name=None)
for sheet_name in surface_data.items():
    data = sheet_name[1]
    # Extract the x and y coordinates from the data
    x = data.get('Samples')
    y1 = data.get('ADXL Magnitude')
    y2 = data.get('IMU Magnitude').

    # Create a figure and axis object
    fig, ax = plt.subplots()

    # Plot the line
    ax.plot(x, y1, label ='ADXL Magnitude')
    ax.plot(x, y2, label = 'IMU Magnitude')

    # Add labels and title
    ax.set_xlabel('Samples')
    ax.set_ylabel('Magnitude')
    ax.legend()
    ax.set_title('Surface acceleration data - '+str(cnt))
    cnt+=1
    
    fig.tight_layout()
    # Display the plot
    plt.show()
