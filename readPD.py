from pyModbusTCP.client import ModbusClient
import time
import pymysql as sql

conn = sql.connect(host='127.0.0.1', user='root', password='ziumks', db='maeil', charset='utf8')
qry = 'insert into monitoring_item_PD(deviceNo, time, Temp_R, Temp_S, Temp_T, PD) values(%s,%s,%s,%s,%s,%s);'

def readPDTemp(num):
	c=ModbusClient(host='192.168.0.4', port=502, unit_id=4, auto_open=True, auto_close=True)
	if c.is_open():
		registerPD = None
		registerTemp = None
	else:
		c.open()
		registerPD = None
		registerTemp = None
	if num == 4:
		registerPD = c.read_holding_registers(reg_addr=450, reg_nb=1)
		registerTemp = c.read_holding_registers(reg_addr=418,reg_nb=3)		

	elif num == 5:
		registerPD = c.read_holding_registers(reg_addr=451, reg_nb=1)
		registerTemp = c.read_holding_registers(reg_addr=421, reg_nb=3)

	elif num == 6:
		registerPD = c.read_holding_registers(reg_addr=452, reg_nb=1)
		registerTemp = c.read_holding_registers(reg_addr=424, reg_nb=3)
	else:
		print("system error!")
	param = num, round(time.time()), registerTemp[0]/10, registerTemp[1]/10, registerTemp[2]/10, registerPD[0]
	if registerPD:
		if registerTemp:
			try:
				with conn.cursor() as cursor:
					cursor.execute(qry, param)
					conn.commit()
			except TypeError:
				print('connection error with Db. Check it.')
				pass
		else:
			print("reboot CAM-4 to get temperature")
	else:
		print("reboot CAM-4 to get PD")
	c.close()

	result = {'i':param[0], 'time':param[1], 'Temp_R':param[2], 'Temp_S':param[3], 'Temp_T':param[4], 'PD':param[5]}
	return result
