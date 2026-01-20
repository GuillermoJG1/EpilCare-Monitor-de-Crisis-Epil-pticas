#	Martínez González Guillermo Josué
#	Quintero Manriquez Alejandra

import network
import time
import camera          
from umqtt.mosquitto import MQTTClient
import machine
import gc 

# === CONFIGURACIÓN ===
SSID = "internet"
PASSWORD = "123456"


MQTT_BROKER = "test.mosquitto.org"
MQTT_CLIENT_ID = "EpilCare" 

# Tópicos
TOPIC_TRIGGER = "tecnm/bio/camara"      
TOPIC_FOTO    = "tecnm/bio/foto/data"   

client = None

# === CONEXIÓN WIFI ===
print("\nConectando a Wi-Fi...")
wlan = network.WLAN(network.STA_IF); wlan.active(True)
wlan.connect(SSID, PASSWORD)
while not wlan.isconnected(): time.sleep(1)
print("Conectado:", wlan.ifconfig()[0])

# === INICIALIZACIÓN CÁMARA ===
try:
    camera.deinit()
    camera.init()
    camera.framesize(8) # 240x240 aprox
except: pass

def tomar_y_enviar_foto():
    global client
    try:
        gc.collect()
        img_bytes = camera.capture()
        if len(img_bytes) > 0:
            print(f"Foto: {len(img_bytes)} bytes")
            # Formato Data URI
            payload = b"data:image/jpeg;base64," + img_base64
            
            print("Enviando MQTT...")
            client.publish(TOPIC_FOTO, payload)
            print("Enviada!")
    except Exception as e: print("Error:", e)
    finally: gc.collect()

def mqtt_callback(topic, msg):
    if topic.decode() == TOPIC_TRIGGER and msg.decode() == "1":
        print("Orden recibida")
        tomar_y_enviar_foto()

def main():
    global client
    client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER)
    client.set_callback(mqtt_callback)
    try:
        client.connect()
        client.subscribe(TOPIC_TRIGGER)
        print("Cámara Lista esperando MQTT...")
    except: return

    while True:
        try: client.check_msg(); time.sleep(0.1) 
        except: 
            try: client.connect(); client.subscribe(TOPIC_TRIGGER)
            except: pass

if __name__ == "__main__":
    main()