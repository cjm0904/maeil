import socket
import struct
import pymysql as sql
import time
import test_crc as crc

conn = sql.connect(host='127.0.0.1', user='root', password='ziumks', db='maeil', charset='utf8')
addr = '192.168.0.6'
port = 2222
buff = 1024

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(1)
s.connect((addr, port))

def process(msg):
	crc.makecrctable()
	msgCrc = crc.crc16(msg, msg.__len__())
	quotient = int(msgCrc/256)
	msgCrcTuple=quotient, msgCrc-(256*quotient)
	msg+=msgCrcTuple
	return msg

def hex2float(num1, num2, num3, num4):
	a = (str(hex((num1 << 8)+num2)))[2:]
	b = (str(hex((num3 << 8)+num4)))[2:]
	try:
		result = struct.unpack('f', struct.pack('i', int(a+b, 16)))[0]
		# result = struct.unpack('f', struct.pack('i', float(a+b)))[0]
		return round(result, 4)
	except struct.error as e:
		print(num1, num2, num3, num4)
       # print(e)
       # result = struct.unpack('f', struct.pack('f', int(a+b, 16)))[0]
		return 0

def gatheringData(num):
	msg = [num, 0x03, 0x2b, 0x05, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	I_R = hex2float(data[19], data[20], data[21], data[22])
	I_S = hex2float(data[23], data[24], data[25], data[26])
	I_T = hex2float(data[27], data[28], data[29], data[30])
	Iavg = hex2float(data[31], data[32], data[33], data[34])
	V_RS = hex2float(data[35], data[36], data[37], data[38])
	V_ST = hex2float(data[39], data[40], data[41], data[42])
	V_TR = hex2float(data[43], data[44], data[45], data[46])
	Vavg_ll = hex2float(data[47], data[48], data[49], data[50])
	P_R = hex2float(data[51], data[52], data[53], data[54])
	P_S = hex2float(data[55], data[56], data[57], data[58])
	P_T = hex2float(data[59], data[60], data[61], data[62])
	P_tot = hex2float(data[63], data[64], data[65], data[66])
	Q_R = hex2float(data[67], data[68], data[69], data[70])
	Q_S = hex2float(data[71], data[72], data[73], data[74])
	Q_T = hex2float(data[75], data[76], data[77], data[78])
	Q_tot = hex2float(data[79], data[80], data[81], data[82])
	S_R = hex2float(data[83], data[84], data[85], data[86])
	S_S = hex2float(data[87], data[88], data[89], data[90])
	S_T = hex2float(data[91], data[92], data[93], data[94])
	
	msg = [num, 0x03, 0x2b, 0x33, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	S_tot = hex2float(data[3], data[4], data[5], data[6])
	pf_R = hex2float(data[7], data[8], data[9], data[10])
	pf_S = hex2float(data[11], data[12], data[13], data[14])
	pf_T = hex2float(data[15], data[16], data[17], data[18])
	pf = hex2float(data[19], data[20], data[21], data[22])
	
	msg = [num, 0x03, 0x2b, 0x8d, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	VTHD_R = hex2float(data[59], data[60], data[61], data[62])
	VTHD_S = hex2float(data[63], data[64], data[65], data[66])
	VTHD_T = hex2float(data[67], data[68], data[69], data[70])
	ITHD_R = hex2float(data[71], data[72], data[73], data[74])
	ITHD_S = hex2float(data[75], data[76], data[77], data[78])
	ITHD_T = hex2float(data[79], data[80], data[81], data[82])
	ITDD_R = hex2float(data[83], data[84], data[85], data[86])
	ITDD_S = hex2float(data[87], data[88], data[89], data[90])
	ITDD_T = hex2float(data[91], data[92], data[93], data[94])
	
	msg = [num, 0x03, 0x2b, 0xe3, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	V_zro = hex2float(data[51], data[52], data[53], data[54])
	I_zro = hex2float(data[55], data[56], data[57], data[58])

	msg = [num, 0x03, 0x2c, 0x0d, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	VUR_ll = hex2float(data[7], data[8], data[9], data[10])
	VUR_zro = hex2float(data[11], data[12], data[13], data[14])
	VUR_ngt = hex2float(data[15], data[16], data[17], data[18])
	IUR = hex2float(data[19], data[20], data[21], data[22])
	IUR_zro = hex2float(data[23], data[24], data[25], data[26])
	IUR_ngt = hex2float(data[27], data[28], data[29], data[30])
	CF_R = hex2float(data[31], data[32], data[33], data[34])
	CF_S = hex2float(data[35], data[36], data[37], data[38])
	CF_T = hex2float(data[39], data[40], data[41], data[42])
	freq = hex2float(data[55], data[56], data[57], data[58])
	
	msg = [num, 0x03, 0x28, 0x8b, 0x00, 0x06]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	Fuzfail_R = data[3] + data[4]
	Fuzfail_S = data[5] + data[6]
	Fuzfail_T = data[7] + data[8]
	PhaseOpen_R = data[9] + data[10]
	PhaseOpen_S = data[11] + data[12]
	PhaseOpen_T = data[13] + data[14]

	msg = [num, 0x03, 0x32, 0xc8, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	firstHV_R = hex2float(data[7], data[8], data[9], data[10])
	thirdHV_R = hex2float(data[15], data[16], data[17], data[18])
	fifthHV_R = hex2float(data[23], data[24], data[25], data[26])
	
	msg = [num, 0x03, 0x33, 0x08, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	firstHV_S = hex2float(data[7], data[8], data[9], data[10])
	thirdHV_S = hex2float(data[15], data[16], data[17], data[18])
	fifthHV_S = hex2float(data[23], data[24], data[25], data[26])

	msg = [num, 0x03, 0x33, 0x48, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	firstHV_T = hex2float(data[7], data[8], data[9], data[10])
	thirdHV_T = hex2float(data[15], data[16], data[17], data[18])
	fifthHV_T = hex2float(data[23], data[24], data[25], data[26])

	msg = [num, 0x03, 0x33, 0x88, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	firstHI_R = hex2float(data[7], data[8], data[9], data[10])
	thirdHI_R = hex2float(data[15], data[16], data[17], data[18])
	fifthHI_R = hex2float(data[23], data[24], data[25], data[26])

	msg = [num, 0x03, 0x33, 0xc8, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	firstHI_S = hex2float(data[7], data[8], data[9], data[10])
	thirdHI_S = hex2float(data[15], data[16], data[17], data[18])
	fifthHI_S = hex2float(data[23], data[24], data[25], data[26])

	msg = [num, 0x03, 0x34, 0x08, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	firstHI_T = hex2float(data[7], data[8], data[9], data[10])
	thirdHI_T = hex2float(data[15], data[16], data[17], data[18])
	fifthHI_T = hex2float(data[23], data[24], data[25], data[26])

	qry = 'insert into monitoring_item_multimeter(deviceNo, time, I_R, I_S, I_T, Iavg, V_RS, V_ST, V_TR, '
	qry +='Vavg_ll, P_R, P_S, P_T, P_tot, Q_R, Q_S, Q_T, Q_tot, S_R, S_S, S_T, S_tot, pf_R, pf_S, pf_T, pf, '
	qry +='VTHD_R, VTHD_S, VTHD_T, ITHD_R, ITHD_S, ITHD_T, ITDD_R, ITDD_S, ITDD_T, V_zro, I_zro, VUR_ll, VUR_ngt, '
	qry +='IUR, IUR_zro, IUR_ngt, CF_R, CF_S, CF_T, freq, Fuzfail_R, Fuzfail_S, Fuzfail_T,PhaseOpen_R, PhaseOpen_S, PhaseOpen_T, firstHV_R, thirdHV_R, fifthHV_R,'
	qry +='firstHV_S, thirdHV_S, fifthHV_S, firstHV_T, thirdHV_T, fifthHV_T, firstHI_R, thirdHI_R, fifthHI_R, firstHI_S, thirdHI_S, fifthHI_S,'
	qry +='firstHI_T, thirdHI_T, fifthHI_T)'
	qry += 'values (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,'
	qry += ' %s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s);'
	
	param = (num, round(time.time()),I_R, I_S, I_T, Iavg, V_RS, V_ST, V_TR, Vavg_ll, P_R, P_S, P_T, P_tot, Q_R, Q_S, Q_T, Q_tot, S_R, S_S, S_T)
	param+= (S_tot, pf_R, pf_S, pf_T, pf, VTHD_R, VTHD_S, VTHD_T, ITHD_R, ITHD_S, ITHD_T, ITDD_R, ITDD_S, ITDD_T, V_zro, I_zro, VUR_ll, VUR_ngt,IUR)
	param+= (IUR_zro, IUR_ngt, CF_R, CF_S, CF_T, freq, Fuzfail_R, Fuzfail_S, Fuzfail_T, PhaseOpen_R, PhaseOpen_S, PhaseOpen_T, firstHV_R, thirdHV_R)
	param+= (fifthHV_R, firstHV_S, thirdHV_S, fifthHV_S, firstHV_T, thirdHV_T, fifthHV_T, firstHI_R, thirdHI_R, fifthHI_R, firstHI_S, thirdHI_S, fifthHI_S, firstHI_T, thirdHI_T, fifthHI_T)
	try:
		with conn.cursor() as cursor:
			cursor.execute(qry, param)
			conn.commit()
	except TypeError as e:
		print(e)
		pass

	result = {'i':param[0], 'time':param[1], 'I_R':param[2], 'I_S':param[3], 'I_T':param[4], 'Iavg':param[5], 'V_RS':param[6], 'V_ST':param[7], 'V_TR':param[8], 'Vavg_ll':param[9], 'P_R':param[10], 'P_S':param[11], 'P_T':param[12], 'P_tot':param[13], 'Q_R':param[14], 'Q_S':param[15], 'Q_T':param[16], 'Q_tot':param[17], 'S_R':param[18], 'S_S':param[19], 'S_T':param[20], 'S_tot':param[21], 'pf_R':param[22], 'pf_S':param[23], 'pf_T':param[24], 'pf':param[25], 'VTHD_R':param[26], 'VTHD_S':param[27], 'VTHD_T':param[28], 'ITHD_R':param[29], 'ITHD_S':param[30], 'ITHD_T':param[31], 'ITDD_R':param[32], 'ITDD_S':param[33], 'ITDD_T':param[34], 'V_zro':param[35], 'I_zro':param[36], 'VUR_ll':param[37], 'VUR_ngt':param[38], 'IUR':param[39], 'IUR_zro':param[40], 'IUR_ngt':param[41], 'CF_R':param[42], 'CF_S':param[43], 'CF_T':param[44], 'freq':param[45], 'Fuzfail_R':param[46], 'Fuzfail_S':param[47], 'Fuzfail_T':param[48], 'PhaseOpen_R':param[49], 'PhaseOpen_S':param[50], 'PhaseOpen_T':param[51], 'firstHV_R':param[52], 'thirdHV_R':param[53], 'fifthHV_R':param[54], 'firstHV_S':param[55], 'thirdHV_S':param[56], 'fifthHV_S':param[57], 'firstHV_T':param[58], 'thirdHV_T':param[59], 'fifthHV_T':param[60], 'firstHI_R':param[61], 'thirdHI_R':param[62], 'fifthHI_R':param[63], 'firstHI_S':param[64], 'thirdHI_S':param[65], 'fifthHI_S':param[66], 'firstHI_T':param[67], 'thirdHI_T':param[68], 'fifthHI_T':param[69]}

	return result

	
