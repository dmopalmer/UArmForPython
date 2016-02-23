# UArmForPython
uArm python library designed by Joey Song ( joey@ufactory.cc / astainsong@gmail.com)



## INSTALLATION

Install this library along with [pyfirmata-uarm](https://github.com/uArm-Developer/pyFirmata). Install this library before you use UArm For Python

In uarm, install [FIRMATA-EEPROM-StandardFirmataEEPROM.ino	](https://github.com/uArm-Developer/FirmataEEPROM/) after you calibrate uarm - Check here to see [How to calibrate](http://developer.ufactory.cc/quickstart/)

Then cd to UArmForPython path in the directory and use **python setup.py install** to install UArmForPython library.



## API REFERENCE

Remember to import the library that:
**from UArmForPython.uarm_python import Uarm**

### 0. Class name

> Uarm()
 
Exameples:
```
uarm = Uarm('/dev/ttyUSB0')
```


### 1.Attach and detach certain servo

>servoAttach(servo_number)
>
>servoDetach(servo_number)

Exameples:
```
uarm.servoAttach(1)
```

### 2. Detach and attch uarm (recommend)
>uarmAttach()
>
>uarmDetach()


### 3. Control certain angle and control 4 angles at the same time
**Need to attach servo first**
 
>writeServoAngle(servo_number,angle) 
>
>writeAngles(servo_1,servo_2,servo_3,servo_4)


Exameples:
```
uarm.writeServoAngle(1,150)
```

### 4. Read current angle
**return type - float**
readAngle(servo_number)

Exameples:
```
print uarm.readAngle(1)
```

### 5. Read current position
**return type - float** 
>currentX()
>
>currentY()
>
>currentZ()
>
>currentCoord()


### 6. All moveTo s	
>moveTo(x,y,z)
>

**add time parameter**

>moveToWithTime(x,y,z,timeSpend)
>

**control servo_4 and time at the same time. servo_4_relative should be 1 or 0**

>moveToWithS4(x,y,z,timeSpend,servo_4_relative,servo_4)

Exameples:
```
uarm.moveTo(10,-15,15)
```

### 7. Pump on and pump off
>pumpOn()

>pumpOff()

### 8. Stopper status
**return type - True/False**
>stopperStatus()

