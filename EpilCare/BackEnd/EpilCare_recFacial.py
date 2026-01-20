#	Martínez González Guillermo Josué
#	Quintero Manriquez Alejandra

import cv2
import numpy as np
import base64
import time

# ==============================
# CONFIGURACIÓN MQTT
# ==============================
MQTT_BROKER = "test.mosquitto.org" 
TOPIC_FOTO = "tecnm/bio/foto/data"

# ==============================
# MODELO DE INTELIGENCIA ARTIFICIAL
# ==============================
print("Cargando modelo de IA...")
try:
    prototxt = "deploy.prototxt"
    model = "rep10_240x240_ssd_itr_17000.caffemodel"
    net = cv2.dnn.repNetFromCaffe(prototxt, model)
    print("Modelo cargado correctamente.")
except Exception as e:
    print(f"Error cargando modelo IA: {e}")
    exit()

# Función para procesar la imagen cuando llega por MQTT
def procesar_imagen(base64_string):
    try:
        # 1. Limpiar el encabezado "data:image/jpeg;base64,"
        if "," in base64_string:
            base64_string = base64_string.split(",")[1]

        # 2. Decodificar Base64 a Bytes
        img_data = base64.b64decode(base64_string)
        
        # 3. Convertir Bytes a Imagen OpenCV
        np_arr = np.frombuffer(img_data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            print("Error: No se pudo decodificar la imagen.")
            return

        # ==============================
        # LÓGICA DE DETECCIÓN FACIAL
        # ==============================
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), (104.0, 177.0, 123.0))

        net.setInput(blob)
        detections = net.forward()

        faces_found = 0

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]

            if confidence > 0.5: # Umbral
                faces_found += 1
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (x1, y1, x2, y2) = box.astype("int")

                # Dibujar rectángulo y texto
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                text = f"{confidence*100:.1f}%"
                y = y1 - 10 if y1 - 10 > 10 else y1 + 10
                cv2.putText(frame, text, (x1, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

        print(f"Imagen procesada: {faces_found} rostros detectados.")
        
        # Mostrar imagen en ventana
        cv2.imshow("Monitor IA - EpilCare", frame)
        cv2.waitKey(1) # Necesario para refrescar la ventana

    except Exception as e:
        print(f"Error procesando imagen: {e}")

# ==============================
# CALLBACKS MQTT
# ==============================
def on_connect(client, userdata, flags, rc):
    print("Conectado al Broker MQTT!")
    client.subscribe(TOPIC_FOTO)
    print(f"Esperando imágenes en: {TOPIC_FOTO}...")

def on_message(client, userdata, msg):
    print("¡Imagen recibida desde ESP32!")
    payload = msg.payload.decode('utf-8')
    procesar_imagen(payload)

# ==============================
# INICIO DEL PROGRAMA
# ==============================
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_forever() # Mantiene el script corriendo esperando mensajes
except KeyboardInterrupt:
    print("Cerrando programa...")
    cv2.destroyAllWindows()
