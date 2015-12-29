
from UArmForPython.uarm_python import Uarm
uarm = Uarm('/dev/ttyUSB0')




inputString = raw_input("connect or not(y/n): ")
if inputString == 'y':
	uarm.moveTo(15,-15,15)
