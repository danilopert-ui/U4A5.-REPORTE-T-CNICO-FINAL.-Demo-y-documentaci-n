# Gemelo Digital del Robot Seguidor de Línea (CPS + IA)

Proyecto integrador de la Unidad 4. Sistema ciberfísico con inteligencia artificial
que adquiere la telemetría de un robot seguidor de línea físico, la procesa, clasifica
su estado de operación con un modelo entrenado y genera una salida accionable.

---

## ¿Qué hace el proyecto?

El robot (ESP32 + sensores QTR-1A) publica su telemetría por MQTT. Un stack en Docker
(Mosquitto + Node-RED + InfluxDB) la recibe y registra. Con esos datos se entrenó un
modelo de clasificación (Random Forest, 94.6% de exactitud) que identifica el estado del
robot: centrado, desviado a la izquierda, desviado a la derecha o recuperación. Un pipeline
de extremo a extremo ejecuta el modelo y produce alertas y recomendaciones de ajuste.

```
Adquisición  ->  Preprocesado  ->  Inferencia  ->  Salida accionable
 (robot/MQTT)     (limpieza)        (modelo IA)      (estado + alerta)
```

---

## Estructura del repositorio

```
seguidor-stack/
├── docker-compose.yml          # Stack: Mosquitto + InfluxDB + Node-RED
├── mosquitto-config/
│   └── mosquitto.conf          # Configuración del broker MQTT
├── firmware/
│   └── seguidor_linea_v3_mqtt.ino   # Código del ESP32 (WiFi STA + MQTT + web)
├── dataset/
│   ├── v0.1/datos_crudos.csv        # Dataset crudo (1338 muestras)
│   └── v0.2/datos_limpios.csv       # Dataset limpio (516 muestras)
├── pipeline/
│   ├── pipeline_e2e.py              # Pipeline E2E (modo csv y modo mqtt)
│   ├── modelo_seguidor.joblib       # Modelo Random Forest entrenado
│   └── datos_limpios_v0.2.csv       # Copia del dataset para el pipeline
├── notebooks/
│   └── U4A3_notebook.ipynb          # Entrenamiento y evaluación del modelo
├── documentacion/
│   ├── U4A1_brief.docx              # Brief técnico
│   ├── U4A2_dataset.docx            # Reporte del dataset
│   ├── U4A3_modelo.docx             # Reporte del modelo
│   ├── U4A4_pipeline.docx           # Reporte del pipeline + ciberhigiene
│   └── U4A5_reporte_final.docx      # Reporte técnico final
└── evidencias/
    └── capturas/                    # Capturas y video de demostración
```

---

## Requisitos

- Docker Desktop (para el stack de datos)
- Python 3.10+ con: `pandas`, `scikit-learn`, `joblib`, `paho-mqtt`
- Arduino IDE con las librerías `QTRSensors` y `PubSubClient` (para el firmware)

```bash
pip install pandas scikit-learn joblib paho-mqtt
```

---

## Cómo ejecutar / revisar el proyecto

### 1. Levantar el stack de datos
```bash
cd seguidor-stack
docker compose up -d
```
Interfaces: Node-RED en http://localhost:1880 e InfluxDB en http://localhost:8086

### 2. Cargar el firmware
Abrir `firmware/seguidor_linea_v3_mqtt.ino` en Arduino IDE, ajustar SSID/contraseña del
hotspot y la IP del broker, y subirlo al ESP32.

### 3. Ejecutar el pipeline del gemelo digital
```bash
cd pipeline
# Demostración con el dataset (sin robot):
python pipeline_e2e.py --modo csv
# Inferencia en vivo desde el robot:
python pipeline_e2e.py --modo mqtt --broker 192.168.137.1
```

### 4. Revisar el entrenamiento del modelo
Abrir `notebooks/U4A3_notebook.ipynb` en Jupyter o Google Colab (colocar el CSV junto al
notebook) y ejecutar las celdas en orden.

---

## Resultados principales

| Modelo | Accuracy | F1-macro |
|--------|----------|----------|
| Baseline (clase mayoritaria) | 0.581 | 0.184 |
| Árbol de decisión | 0.930 | 0.921 |
| KNN (k=5) | 0.946 | 0.932 |
| **Random Forest** (seleccionado) | **0.946** | **0.940** |

---

## Limitaciones y trabajo futuro

- El dataset proviene de una sola pista e iluminación; las clases `desviado_izq` y
  `recuperacion` están sub-representadas.
- En desarrollo: integración de un sensor inercial **MPU6050** (I2C, GPIO 21/22) para que
  el gemelo digital reproduzca el movimiento del robot en tiempo real.

---

## Ciberhigiene

- El broker y la base de datos operan en una red privada (hotspot dedicado), no expuesta a internet.
- Las credenciales (token de InfluxDB, contraseña del hotspot) **no se comparten públicamente**
  y se mantienen fuera del código del robot.
- No publicar en el repositorio: tokens, contraseñas ni las IPs internas de la red.

---

## Equipo

[COMPLETAR: nombres e integrantes del equipo]
