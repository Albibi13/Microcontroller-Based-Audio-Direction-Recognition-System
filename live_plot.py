from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

# initialize serial port
ser = serial.Serial()
ser.port = "COM7"
ser.baudrate = 115200
# ser.timeout = 10 #specify timeout when using readline()
ser.open()
if ser.is_open:
    print("\nAll right, serial port now open. Configuration:\n")
    print(ser, "\n")  # print serial parameters

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []  # time stamps
ys = []  # signal out


# This function is called periodically from FuncAnimation
def animate(i, xs, ys):
    # Acquire and parse data from serial port
    line = ser.read()  # ascii
    # line_as_list = line.split(b',')
    i = int(line[0])
    time_now = datetime.now()
    current_time = time_now.strftime("%S.%f")
    # Add x and y to lists
    xs.append(current_time)
    ys.append(i)

    # Limit x and y lists to 100 items
    xs = xs[-100:]
    ys = ys[-100:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys, label="signal out")

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Sensor output')
    plt.gca().axes.get_xaxis().set_ticks([])
    plt.xlabel('time')
    plt.legend()
    plt.axis([0, None, 0, 256])


# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=100)
plt.show()
