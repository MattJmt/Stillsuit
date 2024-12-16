import pyCandle
import sys
import time

candle = pyCandle.Candle(pyCandle.CAN_BAUD_8M, True, pyCandle.SPI)
ids = candle.ping()

if len(ids) == 0:
    sys.exit("NO DRIVES FOUND")

for id in ids:
    candle.addMd80(id)
    # max torque was 1.5
    candle.writeMd80Register(id, pyCandle.Md80Reg_E.maxTorque, 9.00) # Set maximum torque
    candle.writeMd80Register(id, pyCandle.Md80Reg_E.maxVelocity, 60.0) # Set maximum velocity
    candle.configMd80Can(id, id, pyCandle.CAN_BAUD_8M, 100, False) # Configure CAN bus settings

    # Save everything!
    time.sleep(2)

for id in ids:
    candle.configMd80Save(id)
    time.sleep(2)


sys.exit("EXIT SUCCESS")
