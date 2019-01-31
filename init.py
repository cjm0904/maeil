import vtms100, accura, readPD, rslt, mqtt 
import time, json
import serial

port = 'ttyUSB0'
#/dev/ttyUSB0

s = serial.Serial(port='/dev/ttyUSB0', baudrate=57600, parity=serial.PARITY_NONE ,timeout=None, stopbits=serial.STOPBITS_ONE, xonxoff=True)


while True:
#	PD1 = readPD.readPDTemp(4)
#	PD2 = readPD.readPDTemp(5)
#	PD3 = readPD.readPDTemp(6)
	PD1 = rslt.readPDTemp(4)
	PD2 = rslt.readPDTemp(5)
	PD3 = rslt.readPDTemp(6)
	accu1 = accura.gatheringData(1)
	accu2 = accura.gatheringData(2)
	accu3 = accura.gatheringData(3)
	gas1 = vtms100.gatheringGas(43)
	gas2 = vtms100.gatheringGas(44)
	
	PD = rslt.readpd()
	temp = rslt.readtemp()
	kescoAccura1 = accura.gatheringAverageData(1)
	kescoAccura2 = accura.gatheringAverageData(2)
	kescoAccura3 = accura.gatheringAverageData(3)
	kescoGas1 = vtms100.kescoGatheringGas(43)
	kescoGas2 = vtms100.kescoGatheringGas(44)	
	
	if gas2!=None:
		#data = PD1, PD2, PD3, accu1, accu2, accu3, gas1, gas2
		re = str(PD1).replace(" ","") + '\n' + str(PD2).replace(" ","") + '\n' + str(PD3).replace(" ","") + '\n' + str(accu1).replace(" ","") + '\n' + str(accu2).replace(" ","") + '\n' + str(accu3).replace(" ","") +'\n' + str(gas1).replace(" ","") + '\n' + str(gas2).replace(" ","") + '\n'

	else:
		#data = PD1, PD2, PD3, accu1, accu2, accu3, gas1
		re = str(PD1).replace(" ","") + '\n' + str(PD2).replace(" ","") + '\n' + str(PD3).replace(" ","") + '\n' + str(accu1).replace(" ","") + '\n' + str(accu2).replace(" ","") + '\n' + str(accu3).replace(" ","") +'\n' + str(gas1).replace(" ","") + '\n'
	print(re)
	s.write(re.encode())

	
	if kescoGas2 != None:
		data = PD, temp, kescoAccura1, kescoAccura2, kescoAccura3, kescoGas1, kescoGas2
	else:
		data = PD, temp, kescoAccura1, kescoAccura2, kescoAccura3, kescoGas1

	
	try:
		for i in range(0, data.__len__()):
			mqtt.mq_client.publish(topic='rmms/maeil', payload=json.dumps(data[i]), qos=0)
	except ConnectionError as e:
		print(e)
		pass
	time.sleep(5)
	
