import time
import numpy as np
from L1_lidar import Lidar   # import your existing module


class ObstacleAvoidance:
    def __init__(self):
        self.lidar = Lidar()

        # Tunable parameters
        self.safe_distance = 0.8   # meters
        self.front_angle = 120      # degrees (field of view)

    def start(self):
        self.lidar.connect()
        self.processor = self.lidar.run()
        time.sleep(1)

    def stop(self):
        self.lidar.kill(self.processor)

    def detect_obstacle(self, data):
        
        #Detect if obstacle exists in front sector
        
        # Removes angle from data
        distances = [d[0] for d in data]

        if distances is None:
            return False, None

        if len(distances) == 0:
            return False, None

        min_distance = min(distances)
        
        if min_distance < self.safe_distance:
            return True, min_distance
        else:
            return False, min_distance

    def decide_direction(self, data):
        
        #Decide where to go: LEFT, RIGHT, or FORWARD
       
        left = []
        right = []

        for d, angle in data:
            if d <= 0:
                continue

            if angle < 0:
                left.append(d)
            else:
                right.append(d)

        left_avg = np.mean(left) if left else 0
        right_avg = np.mean(right) if right else 0

        if left_avg < right_avg:
            return "LEFT"
        else:
            return "RIGHT"

    def run(self):
        try:
            #while True:        We dont' want it to loop when running inside other code
                time.sleep(0.2)

                data = self.lidar.get()

                # Filter only front angles
                front_data = []
                for d, angle in data:
                    if -self.front_angle/2 <= angle <= self.front_angle/2:
                        if d > 0:  # ignore invalid (-1)
                            front_data.append([d, angle])

                obstacle, dist = self.detect_obstacle(front_data)

                if obstacle:
                    direction = self.decide_direction(front_data)
                    print(f"Obstacle at {dist:.2f} m. TURN {direction}")
                    return direction
                else:
                    print("Path is clear. MOVE FORWARD")
                    return False

        except KeyboardInterrupt:
            print("Stopping...")
            self.stop()


if __name__ == "__main__":
    robot = ObstacleAvoidance()
    robot.start()
    robot.run()