import socket
import struct
import pymysql as sql
import time
import test_crc as crc

conn = sql.connect(host='127.0.0.1', user='root', password='ziumks', db='maeil',charset='utf8')
addr='192.168.0.7'
port=2222
buff=1024

s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(1)
s.connect((addr, port))
def process(msg):
	crc.makecrctable()
	msgCrc = crc.crc16(msg, msg.__len__())
	quotient = int(msgCrc/16**2)
	msgCrcTuple=quotient, msgCrc - (256*quotient)
	msg+=msgCrcTuple
	return msg

def hex2float(num1, num2, num3, num4):
	try:
		a = (str(hex((num1 << 8)+num2)))[2:]
		b = (str(hex((num3 << 8)+num4)))[2:]
		result = struct.unpack('f', struct.pack('i', int(b+a, 16)))[0] #리틀엔디언이라 순서 바꿔야함.
		return round(result, 4)
	except struct.error as e:
		print(e)
		return 0

def gatheringGas(num):
	try:
		msg = [num, 0x03, 0x00, 0x19, 0x00, 0x02]
		msg = process(msg)
		s.send(bytes(msg))
		data = s.recv(buff)
		if not data:
			pass
		if data.__len__()!=9:
			temperature=0
			pass
		else:
			temperature=hex2float(data[3], data[4], data[5], data[6])
		msg = [num, 0x03, 0x00, 0x33, 0x00, 0x04]
		msg = process(msg)
		s.send(bytes(msg))
		if data.__len__()!=13:
			waterContent=0
		else:
			waterContent=hex2float(data[7], data[8], data[9], data[10])
		qry = 'insert into monitoring_item_trans(deviceNo, time, temperature, waterContent)'
		qry += 'values(%s, %s, %s, %s)'
		param = (num, round(time.time()), temperature, waterContent)
		
		try:
			with conn.cursor() as cursor:
				cursor.execute(qry, param)
				conn.commit()
		except TypeError as e:
			print('error connection with DB')
			pass
		result = {'i':param[0], 'time':param[1], 'temperature':param[2], 'waterContent':param[3]}
#		print(result)
		return result
	except Exception as e:
		print("timeout error")
#		print(e)
		pass

def kescoGatheringGas(num):

	try:
		msg = [num, 0x03, 0x00, 0x19, 0x00, 0x02]  
		msg = process(msg)
		s.send(bytes(msg))        
		data = s.recv(buff)
		if not data:
			pass
		if data.__len__() !=9:
			temperature = 0
			pass
		else:
			temperature = hex2float(data[3], data[4], data[5], data[6])

		msg = [num, 0x03, 0x00, 0x33, 0x00, 0x04]  
		msg = process(msg)
		s.send(bytes(msg))
		data = s.recv(buff)
		if data.__len__()!=13:
			waterActivity = 0
			waterContent = 0
		else:
			waterActivity = hex2float(data[3], data[4], data[5], data[6])
			waterContent = hex2float(data[7], data[8], data[9], data[10])
		msg = [num, 0x03, 0x00, 0x51, 0x00, 0x02] 
		msg = process(msg)
		s.send(bytes(msg))
		data = s.recv(buff)
		if data.__len__() != 9:
			hydrogenR = 0
		else:
			hydrogenR = hex2float(data[3], data[4], data[5], data[6])
		param = (num, int(round(time.time()*1000)), temperature, waterActivity, waterContent, hydrogenR)
		result = {'s':'chungyang_maeil', 'i': 'ml-vtms100-' + str(param[0]), 't': param[1], 'm':{ 'temperature': param[2], 'waterAcitivy': param[3],
              'waterContent': param[4], 'hydrogenResistance': param[5]}
              }
		return result
	except Exception as e:
		print ("timeout error")
		pass

def temp():
	try:
		msg = [44, 0x03, 0x00, 0x19, 0x00, 0x02]
		msg = process(msg)
		msg = s.send(bytes(msg))
		data = s.recv(buff)
		if not data:
			pass
		if data.__len__() != 9:
			temperature = 0
			pass
		else:
			temperature = hex2float(data[3], data[4], data[5], data[6])
			tmp = int(temperature) * 10
		return tmp
	except Exception as e:
		return 0
