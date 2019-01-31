import paho.mqtt.client as mqtt

#brk_ip = '115.90.42.36'

brk_ip = '114.205.0.130'
#brk_ip = '14.63.175.204'
#brk_ip = '121.162.206.130'

#brk_port = 22

#brk_port = 1883
brk_port = 8883
brk_keepalive = 60
tls_addr = '/home/zium/maeil/maeilDas/cacert.pem'


mq_client = mqtt.Client()
mq_client.tls_set(tls_addr)
mq_client.tls_insecure_set(True)
mq_client.max_inflight_messages_set(20)
mq_client.message_retry_set(5)
try:
    mq_client.connect(brk_ip, brk_port, brk_keepalive)
    mq_client.reconnect()
except ConnectionError:
    print("cannot connect with MQTT Server")

