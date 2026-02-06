# Imports code to read from the battery
import L1_ina as bat
import L1_log as log
import time



while True:
    current_voltage = bat.readVolts()
    log.tmpFile(current_voltage, "voltage")
    time.sleep(1)
