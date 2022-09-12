import serial
from serial.tools import list_ports

port = list(serial.tools.list_ports.comports())
portName =""
for p in port:
	# print(p.device)
	#print(p.description)
	if str.__contains__(p.description,"Digilent"):
		portName = (p.device)
