import L1_lidar as lidar
import L2_vector as vect
import L2_compass_heading as comp
import time
import L1_log as log

SENSOR_IP = "192.168.6.221"

# Instantiate the Lidar object
lidarsensor = lidar.Lidar(IP=SENSOR_IP)

# Connect to the Lidar device
lidarsensor.connect()

# Start the Lidar thread
processor = lidarsensor.run()
time.sleep(1)  # Allow some time for the thread to start

# Continuously retrieve and print the Lidar data
try:
    while True:
        time.sleep(0.5)
        dist, angle = vect.getNearest(lidarsensor.get())
        log.tmpFile(dist, "Distance")
        log.tmpFile(angle, "Angle")
        print(dist, angle)

        x, y = vect.polar2cart(dist, angle)
        log.tmpFile(x, "X")
        log.tmpFile(y, "Y")
        print(x, y)
except KeyboardInterrupt:
    print("Stopping Lidar...")
finally:
    # Kill the Lidar thread when done
    lidarsensor.kill(processor)