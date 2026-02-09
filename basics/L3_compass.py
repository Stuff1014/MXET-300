import L2_compass_heading as compass
import L1_log as log
import L1_ina as bat
import time


while True:
    # Get current voltage
    current_voltage = bat.readVolts()
    log.tmpFile(current_voltage, "voltage")
    time.sleep(1)

    # Gets heading in degrees
    direction = compass.get_heading()
    log.tmpFile(direction, "direction")

    # Converts to compass heading
    if (-22.5 <= direction < 22.5):
        log.stringTmpFile("North", "heading")
    elif (22.5 <= direction < 67.5):
        log.stringTmpFile("Northeast", "heading")
    elif (67.5 <= direction < 112.5):
        log.stringTmpFile("East", "heading")
    elif (112.5 <= direction < 157.5):
        log.stringTmpFile("Southeast", "heading")
    elif (157.5 <= direction) or (direction < -157.5):
        log.stringTmpFile("South", "heading")
    elif (-157.5 <= direction < -112.5):
        log.stringTmpFile("Southwest", "heading")
    elif (-112.5 <= direction < -67.5):
        log.stringTmpFile("West", "heading")
    elif (-67.5 <= direction < -22.5):
        log.stringTmpFile("Northwest", "heading")
    else:
        log.stringTmpFile("BOOM", "heading")
    time.sleep(0.1)