import L2_kinematics as kin
import L1_log as log
import L1_ina as bat
import time

while True:
    pdl, pdr = kin.getPdCurrent()
    x_dot, theta_dot = kin.getMotion()

    log.tmpFile(pdl, "pdl")
    log.tmpFile(pdr, "pdr")
    log.tmpFile(x_dot, "x_dot")
    log.tmpFile(theta_dot, "theta_dot")

    print("pdl =", pdl)
    print("pdr =", pdr)

    print("x_dot =", x_dot)
    print("theta_dot =", theta_dot)

    current_voltage = bat.readVolts()
    log.tmpFile(current_voltage, "voltage")

    time.sleep(0.2)