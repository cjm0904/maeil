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
#		print(num1, num2, num3, num4)
		pass
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

def gatheringAverageData(num):

	msg = [num, 0x03, 0x28, 0x8b, 0x00, 0x06]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	failPhaseA = data[3] + data[4]
	failPhaseB = data[5] + data[6]
	failPhaseC = data[7] + data[8]
	openPhaseA = data[9] + data[10]
	openPhaseB = data[11] + data[12]
	openPhaseC = data[13] + data[14]


	msg = [num, 0x03, 0x2b, 0x05, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	Van = hex2float(data[3], data[4], data[5], data[6])
	Vbn = hex2float(data[7], data[8], data[9], data[10])
	Vcn = hex2float(data[11], data[12], data[13], data[14])
	Vavg = hex2float(data[15], data[16], data[17], data[18])
	Ia = hex2float(data[19], data[20], data[21], data[22])
	Ib = hex2float(data[23], data[24], data[25], data[26])
	Ic = hex2float(data[27], data[28], data[29], data[30])
	Iavg = hex2float(data[31],data[32], data[33], data[34])
	Vab = hex2float(data[35], data[36], data[37], data[38])
	Vbc = hex2float(data[39], data[40], data[41], data[42])
	Vca = hex2float(data[43], data[44], data[45], data[46])
	Vavg_II = hex2float(data[47], data[48], data[49], data[50])
	Pa = hex2float(data[51], data[52], data[53], data[54])
	Pb = hex2float(data[55], data[56], data[57], data[58])
	Pc = hex2float(data[59], data[60], data[61], data[62])
	Ptot = hex2float(data[63], data[64], data[65], data[66])
	Qa = hex2float(data[67], data[68], data[69], data[70])
	Qb = hex2float(data[71], data[72], data[73], data[74])
	Qc = hex2float(data[75], data[76], data[77], data[78])
	Qtot = hex2float(data[79], data[80], data[81], data[82])
	Sa = hex2float(data[83], data[84], data[85], data[86])
	Sb = hex2float(data[87], data[88], data[89], data[90])
	Sc = hex2float(data[91], data[92], data[93], data[94])

	msg = [num, 0x03, 0x2b, 0x33, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	Stot = hex2float(data[3], data[4], data[5], data[6])
	PFa = hex2float(data[7], data[8], data[9], data[10]) * 100
	PFb = hex2float(data[11], data[12], data[13], data[14]) * 100
	PFc = hex2float(data[15], data[16], data[17], data[18]) * 100
	PFtot = hex2float(data[19], data[20], data[21], data[22]) * 100
	angleA = data[24]
	angleB = data[26]
	angleC = data[28]
	angleTot = data[30]

	msg = [num, 0x03, 0x2b, 0x41, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)

	RKwh = hex2int(data[3], data[4], data[5], data[6])
	DKwh = hex2int(data[7], data[8], data[9], data[10])
	SumKwh = hex2int(data[11], data[12], data[13], data[14])
	NetKwh = hex2int(data[15], data[16], data[17], data[18])
	RKVARh = hex2int(data[19], data[20], data[21], data[22])
	DKVARh = hex2int(data[23], data[24], data[25], data[26])
	SumKVARh = hex2int(data[27], data[28], data[29], data[30])
	NetKVARh = hex2int(data[31], data[32], data[33], data[34])
	KVAh = hex2int(data[35], data[36], data[37], data[38])
	RKWhA = hex2int(data[39], data[40], data[41], data[42])
	RKWhB = hex2int(data[43], data[44], data[45], data[46])
	RKWhC = hex2int(data[47], data[48], data[49], data[50])
	DKWhA = hex2int(data[51], data[52], data[53], data[54])
	DKWhB = hex2int(data[55], data[56], data[57], data[58])
	DKWhC = hex2int(data[59], data[60], data[61], data[62])
	RKVARhA = hex2int(data[63], data[64], data[65], data[66])
	RKVARhB = hex2int(data[67], data[68], data[69], data[70])
	RKVARhC = hex2int(data[71], data[72], data[73], data[74])
	DKVARhA = hex2float(data[75], data[76], data[77], data[78])
	DKVARhB = hex2float(data[79], data[80], data[81], data[82])
	DKVARhC = hex2float(data[83], data[84], data[85], data[86])
	KVAhA = hex2float(data[87], data[88], data[89], data[90])
	KVAhB = hex2float(data[91], data[92], data[93], data[94])

	msg = [num, 0x03, 0x2b, 0x6f, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)

	KVAhC = hex2float(data[3], data[4], data[5], data[6])
	DemandPa = hex2float(data[7], data[8], data[9], data[10])
	DemandPb = hex2float(data[11], data[12], data[13], data[14])
	DemandPc = hex2float(data[15], data[16], data[17], data[18])
	DemandPtot = hex2float(data[19], data[20], data[21], data[22])
	preDemandPtot = hex2float(data[23], data[24], data[25], data[26])
	DemandQa = hex2float(data[27], data[28], data[29], data[30])
	DemandQb = hex2float(data[31], data[32], data[33], data[34])
	DemandQc = hex2float(data[35], data[36], data[37], data[38])
	DemandKVARtot = hex2float(data[39], data[40], data[41], data[42])
	preDemandQtot = hex2float(data[43], data[44], data[45], data[46])
	DemandSa = hex2float(data[47], data[48], data[49], data[50])
	DemandSb = hex2float(data[51], data[52], data[53], data[54])
	DemandSc = hex2float(data[55], data[56], data[57], data[58])
	DemandStot = hex2float(data[59], data[60], data[61], data[62])

	msg = [num, 0x03, 0x2b, 0x8d, 0x00, 0x50]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)

	preDemandStot = hex2float(data[3], data[4], data[5], data[6])
	DemandIa = hex2float(data[7], data[8], data[9], data[10])
	DemandIb = hex2float(data[11], data[12], data[13], data[14])
	DemandIc = hex2float(data[15], data[16], data[17], data[18])
	DemandIavg = hex2float(data[19], data[20], data[21], data[22])
	preDemandIavg = hex2float(data[23], data[24], data[25], data[26])
	Vanl = hex2float(data[27], data[28], data[29], data[30])
	Vbnl = hex2float(data[31], data[32], data[33], data[34])
	Vcnl = hex2float(data[35], data[36], data[37], data[38])
	Vavgl = hex2float(data[39], data[40], data[41], data[42])
	Ial = hex2float(data[43], data[44], data[45], data[46])
	Ibl = hex2float(data[47], data[48], data[49], data[50])
	Icl = hex2float(data[51], data[52], data[53], data[54])
	Iavgl = hex2float(data[55], data[56], data[57], data[58])
	VTHDA = hex2float(data[59], data[60], data[61], data[62])
	VTHDB = hex2float(data[63], data[64], data[65], data[66])
	VTHDC = hex2float(data[67], data[68], data[69], data[70])
	CTHDA = hex2float(data[71], data[72], data[73], data[74])
	CTHDB = hex2float(data[75], data[76], data[77], data[78])
	CTHDC = hex2float(data[79], data[80], data[81], data[82])
	CTDDA = hex2float(data[83], data[84], data[85], data[86])
	CTDDB = hex2float(data[87], data[88], data[89], data[90])
	CTDDC = hex2float(data[91], data[92], data[93], data[94])

	msg = [num, 0x03, 0x2b, 0xe3, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)

	Vax = hex2float(data[3], data[4], data[5], data[6])
	Vay = hex2float(data[7], data[8], data[9], data[10])
	Vbx = hex2float(data[11], data[12], data[13], data[14])
	Vby = hex2float(data[15], data[16], data[17], data[18])
	Vcx = hex2float(data[19], data[20], data[21], data[22])
	Vcy = hex2float(data[23], data[24], data[25], data[26])
	Iax = hex2float(data[27], data[28], data[29], data[30])
	Iay = hex2float(data[31], data[32], data[33], data[34])
	Ibx = hex2float(data[35], data[36], data[37], data[38])
	Iby = hex2float(data[39], data[40], data[41], data[42])
	Icx = hex2float(data[43], data[44], data[45], data[46])
	Icy = hex2float(data[47], data[48], data[49], data[50])
	ZeroVoltage = hex2float(data[51], data[52], data[53], data[54])
	ZeroCurrent =  hex2float(data[55], data[56], data[57], data[58])

	msg = [num, 0x03, 0x2c, 0x0d, 0x00, 0x30]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)

	Vln = hex2float(data[3], data[4], data[5], data[6])
	Vll = hex2float(data[7], data[8], data[9], data[10])
	ZeroVoltageUnbal = hex2float(data[11], data[12], data[13], data[14])
	NegaVoltageUnbal = hex2float(data[15], data[16], data[17], data[18])
	CurrUnbal = hex2float(data[19], data[20], data[21], data[22])
	ZeroCurrentUnbal = hex2float(data[23], data[24], data[25], data[26])
	NegaCurrentUnbal =  hex2float(data[27], data[28], data[29], data[30])
	CFa = hex2float(data[31], data[32], data[33], data[34])
	CFb = hex2float(data[35], data[36], data[37], data[38])
	CFc = hex2float(data[39], data[40], data[41], data[42])
	KFa = hex2float(data[43], data[44], data[45], data[46])
	KFb = hex2float(data[47], data[48], data[49], data[50])
	KFc = hex2float(data[51], data[52], data[53], data[54])

	msg = [num, 0x03, 0x28, 0x8b, 0x00, 0x06]
	msg = process(msg)
	s.send(bytes(msg))
	data = s.recv(buff)
	failPhaseA = data[3] + data[4]
	failPhaseB = data[5] + data[6]
	failPhaseC = data[7] + data[8]
	openPhaseA = data[9] + data[10]
	openPhaseB = data[11] + data[12]
	openPhaseC = data[13] + data[14]

	param = (num, int(round(time.time()*1000)), Van, Vbn, Vcn, Vavg, Ia, Ib, Ic, Iavg, Vab, Vbc, Vca, Vavg_II, Pa, Pb, Pc, Ptot, Qa, Qb, Qc, Qtot)
	param+= (Sa, Sb, Sc, Stot, PFa, PFb, PFc, PFtot, angleA, angleB, angleC, angleTot, RKwh, DKwh, SumKwh, NetKwh, RKVARh, DKVARh, SumKVARh, NetKVARh)
	param+= (KVAh, RKWhA, RKWhB, RKWhC, DKWhA, DKWhB, DKWhC, RKVARhA, RKVARhB, RKVARhC, DKVARhA, DKVARhB, DKVARhC, KVAhA, KVAhB, KVAhC, DemandPa, DemandPb, DemandPc)
	param+= (DemandPtot, preDemandPtot, DemandQa, DemandQb, DemandQc, DemandKVARtot, preDemandQtot, DemandSa, DemandSb, DemandSc, DemandStot, preDemandStot)
	param+= (DemandIa, DemandIb, DemandIc, DemandIavg, preDemandIavg, Vanl, Vbnl, Vcnl, Vavgl, Ial, Ibl, Icl, Iavgl, VTHDA, VTHDB, VTHDC, CTHDA, CTHDB, CTHDC)
	param+= (CTDDA, CTDDB, CTDDC, Vax, Vay, Vbx, Vby, Vcx, Vcy, Iax, Iay, Ibx, Iby, Icx, Icy, ZeroVoltage, ZeroCurrent, Vln, Vll, ZeroVoltageUnbal, NegaVoltageUnbal)
	param+= (CurrUnbal, ZeroCurrentUnbal, NegaCurrentUnbal, CFa, CFb, CFc, KFa, KFb, KFc)


	result = {'s': 'chungyang_maeil', 'i': 'ml-3300-' + str(param[0]), 't' : param[1], 'm': {'failPhaseA':failPhaseA, 'failPhaseB':failPhaseB, 'failPhaseC':failPhaseC, 'openPhaseA':openPhaseA, 'openPhaseB':openPhaseB, 'openPhaseC':openPhaseC  ,Van: param[2], 'Vbn': param[3], 'Vcn' : param[4], 'Vavg_ln' : param[5], 'Ia' : param[6], 'Ib': param[7], 'Ic': param[8],
              'Iavg': param[9], 'Vab': param[10], 'Vbc': param[11], 'Vca': param[12], 'Vavg_ll': param[13], 'Pa':param[14], 'Pb':param[15], 'Pc':param[16],
              'Ptot':param[17], 'Qa':param[18], 'Qb':param[19], 'Qc': param[20], 'Qtot' : param[21], 'Sa':param[22], 'Sb':param[23], 'Sc':param[24], 'Stot':param[25],
              'PFa' : param[26], 'PFb': param[27], 'PFc': param[28], 'PFtot':param[29], 'angleA':param[30], 'angleB':param[31], 'angleC':param[32], 'angleTot':param[33],
              'RKwh':param[34], 'DKwh': param[35], 'SumKwh':param[36], 'NetKwh':param[37], 'RKVARh':param[38], 'DKVARh':param[39], 'SumKVARh':param[40], 'NetKVARh':param[41],
              'KVAh':param[42], 'RKWhA':param[43], 'RKWhB':param[44], 'RKWhC':param[45], 'DKWhA':param[46], 'DKWhB':param[47], 'DKWhC':param[48], 'RKVARhA':param[49],
              'RKVARhB': param[50], 'RKVARhC':param[51], 'DKVARhA':param[52], 'DKVARhB':param[53], 'DKVARhC':param[54], 'KVAhA':param[55], 'KVAhB': param[56], 'KVAhC':param[57],
              'DemandPa':param[58], 'DemandPb':param[59], 'DemandPc':param[60], 'DemandPtot':param[61], 'preDemandPtot':param[62], 'DemandQa':param[63],'DemandQb':param[64],
              'DemandQc': param[65], 'DemandKVARtot':param[66], 'preDemandQtot':param[67], 'DemandSa': param[68], 'DemandSb':param[69], 'DemandSc':param[70], 'DemandStot':param[71],
              'preDemandStot':param[72], 'DemandIa':param[73], 'DemandIb':param[74], 'DemandIc':param[75], 'DemandIavg':param[76], 'preDemandIavg':param[77], 'Vanl':param[78],
              'Vbnl':param[79], 'Vcnl':param[80], 'Vavgl':param[81], 'Ial':param[82], 'Ibl':param[83], 'Icl':param[84],  'Iavgl': param[85], 'VTHDa':param[86], 'VTHDb':param[87],
              'VTHDc' : param[88], 'ITHDa':param[89], 'ITHDb':param[90], 'ITHDc':param[91], 'ITDDa':param[92], 'ITDDb':param[93], 'ITDDc':param[94], 'Vax':param[95],'Vay':param[96],
              'Vbx': param[97], 'Vby':param[98], 'Vcx':param[99], 'Vcy':param[100], 'Iax':param[101], 'Iay':param[102],'Ibx':param[103], 'Iby':param[104],'Icx':param[105], 'Icy':param[106], 'Vzro':param[107],
              'Izro': param[108], 'Vln':param[109], 'Vll':param[110], 'ZeroVoltageUnbal':param[111], 'NegaVoltageUnbal': param[112], 'CurrUnbal':param[113], 'ZeroCurrentUnbal': param[114],
              'NegaCurrentUnbal':param[115], 'CFa':param[116],' CFb': param[117], 'CFc': param[118], 'KFa': param[119], 'KFb': param[120], 'KFc': param[121], 'failPhaseA': failPhaseA, 'failPhaseB':failPhaseB,
              'failPhaseC':failPhaseC, 'openPhaseA':openPhaseA, 'openPhaseB':openPhaseB, 'openPhaseC':openPhaseC
              }}

	return result

def hex2int(num1, num2, num3, num4):
	answer = (num1<<24) + (num2<<16) + (num3<<8) + num4
	return answer
