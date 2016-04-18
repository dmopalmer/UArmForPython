import time
import serial.tools.list_ports as list_ports
import matplotlib.pyplot as plt
import numpy as np

from UArmForPython.uarm_python import Uarm

# uarm = Uarm('/dev/ttyUSB0')
uarm = Uarm('/dev/tty.usbserial-A6031MM6', claw=False)

for s in uarm.servos():
    print("Servo parameters: {} off:{:.1f}, a:{:.1f}, b:{:.2f}"
          .format(s.servonumber, s.offset, s.data_a, s.data_b))

uarm.moveTo(15,-15,15)
uarm.writeAngle(30,30,30,30,30)

angles = np.arange(0.0,181,5)
nangles = len(angles)
nservos = len(uarm.servos())

anavalues = np.zeros(shape = (nangles, nservos+1), dtype=np.float)
degvalues = np.zeros(shape = (nangles, nservos+1), dtype=np.float)


fig,axes = plt.subplots(1,3, figsize=(20,10))

for s in uarm.servos():
    for i in range(nangles):
        s.writeAngle(angles[i], raw=True)
        time.sleep(2)
        an = s.readAnalog()
        anavalues[i, s.servonumber] = an
        degvalues[i, s.servonumber] = s.readToAngle(an)

    uarm.writeAngle(30, 30, 30, 30, 30)
    time.sleep(2)
    axes[0].plot(angles, anavalues[:, s.servonumber])
    axes[1].plot(angles, degvalues[:, s.servonumber])
    axes[2].plot(angles, degvalues[:, s.servonumber] - angles)

uarm.uarmDetach()

plt.show()


