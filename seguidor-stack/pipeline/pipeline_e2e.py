"""
============================================================
 PIPELINE E2E - GEMELO DIGITAL DEL ROBOT SEGUIDOR DE LINEA
 Actividad U4A4 - Sistema CPS + IA
============================================================

 Flujo:  Adquisicion -> Preprocesado -> Inferencia -> Salida accionable

 El robot publica su telemetria; este pipeline la toma, la limpia,
 ejecuta el modelo de clasificacion (entrenado en U4A3) y produce
 una SALIDA ACCIONABLE: clasifica el estado y, si detecta un problema,
 emite una alerta o recomendacion de ajuste para el robot.

 Dos modos de ejecucion:
   --modo csv    : reproduce el dataset CSV (para demo sin robot)
   --modo mqtt   : se suscribe en vivo al robot por MQTT

 Uso:
   python pipeline_e2e.py --modo csv
   python pipeline_e2e.py --modo mqtt --broker 192.168.137.1

 Requisitos:
   pip install pandas scikit-learn joblib paho-mqtt
============================================================
"""

import argparse
import os
import time
import json
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
import joblib

# ------------------------------------------------------------
# Configuracion
# ------------------------------------------------------------
DATASET   = "datos_limpios_v0.2.csv"
MODELO    = "modelo_seguidor.joblib"
FEATURES  = ["s0_cal", "s1_cal", "s2_cal", "s3_cal"]
TARGET    = "estado"
SEMILLA   = 42

# MQTT (solo en modo mqtt)
MQTT_TOPIC = "seguidor/telemetria"
MQTT_PORT  = 1883


# ------------------------------------------------------------
# ETAPA 0 - Entrenar / cargar el modelo (componente de IA)
# ------------------------------------------------------------
def obtener_modelo():
    """Carga el modelo si existe; si no, lo entrena desde el dataset y lo guarda."""
    if os.path.exists(MODELO):
        print(f"[MODELO] Cargando modelo entrenado desde {MODELO}")
        return joblib.load(MODELO)

    print(f"[MODELO] No existe {MODELO}. Entrenando desde {DATASET}...")
    df = pd.read_csv(DATASET)
    X, y = df[FEATURES], df[TARGET]
    X_train, _, y_train, _ = train_test_split(
        X, y, test_size=0.25, random_state=SEMILLA, stratify=y
    )
    modelo = RandomForestClassifier(n_estimators=100, random_state=SEMILLA)
    modelo.fit(X_train, y_train)
    joblib.dump(modelo, MODELO)
    print(f"[MODELO] Modelo entrenado y guardado en {MODELO}")
    return modelo


# ------------------------------------------------------------
# ETAPA 2 - Preprocesado
# ------------------------------------------------------------
def preprocesar(muestra: dict):
    """
    Recibe una muestra cruda (dict con la telemetria) y devuelve
    el vector de entrada para el modelo, o None si la muestra es invalida.

    Preprocesado aplicado:
      - Selecciona solo las columnas de sensores (s0..s3).
      - Verifica que no falten valores (limpieza de vacios).
      - Verifica el rango valido (0..1000) de los sensores calibrados.
    """
    # Los sensores pueden venir como lista "s":[..] (MQTT) o columnas s0_cal.. (CSV)
    if "s" in muestra and isinstance(muestra["s"], list) and len(muestra["s"]) == 4:
        sensores = muestra["s"]
    else:
        try:
            sensores = [muestra[f] for f in FEATURES]
        except KeyError:
            return None

    # Limpieza: descartar si hay vacios
    if any(v is None for v in sensores):
        return None

    # Filtrado: rango valido
    sensores = [max(0, min(1000, int(v))) for v in sensores]
    return sensores


# ------------------------------------------------------------
# ETAPA 3 + 4 - Inferencia y salida accionable
# ------------------------------------------------------------
def inferir_y_accionar(modelo, sensores, contador=None):
    """
    Ejecuta el modelo sobre la muestra preprocesada y genera la
    SALIDA ACCIONABLE: estado + alerta/recomendacion segun el caso.
    """
    df_in = pd.DataFrame([sensores], columns=FEATURES)
    estado = modelo.predict(df_in)[0]

    # --- Salida accionable: traducir el estado a una accion/alerta ---
    if estado == "centrado":
        nivel = "OK"
        mensaje = "Robot centrado en la linea. Operacion normal."
    elif estado == "desviado_der":
        nivel = "AVISO"
        mensaje = "Desviacion a la DERECHA. Sugerencia: el robot tiende a la derecha; revisar balance de motores o subir Kp."
    elif estado == "desviado_izq":
        nivel = "AVISO"
        mensaje = "Desviacion a la IZQUIERDA. Sugerencia: el robot tiende a la izquierda; revisar balance de motores o subir Kp."
    elif estado == "recuperacion":
        nivel = "ALERTA"
        mensaje = "Robot en RECUPERACION (perdio la linea). Accion: verificar trazado/velocidad en curvas cerradas."
    else:
        nivel = "INFO"
        mensaje = f"Estado: {estado}"

    return {"contador": contador, "sensores": sensores, "estado": estado,
            "nivel": nivel, "mensaje": mensaje}


def imprimir_salida(r):
    """Muestra la salida accionable de forma legible (indicador de estado)."""
    icono = {"OK": "[ OK   ]", "AVISO": "[ AVISO]", "ALERTA": "[ALERTA]", "INFO": "[ INFO ]"}.get(r["nivel"], "[      ]")
    cont = f"#{r['contador']:<4}" if r["contador"] is not None else "     "
    print(f"{icono} {cont} sensores={r['sensores']}  ->  {r['estado']:<13} | {r['mensaje']}")


# ------------------------------------------------------------
# MODO CSV - reproduce el dataset (demostracion sin robot)
# ------------------------------------------------------------
def modo_csv(modelo, limite=25, pausa=0.0):
    print("\n" + "=" * 70)
    print(" MODO CSV-REPLAY  (Adquisicion simulada desde el dataset)")
    print("=" * 70)
    df = pd.read_csv(DATASET)

    # Tomamos una muestra variada de filas (no solo las primeras)
    muestras = df.sample(n=min(limite, len(df)), random_state=SEMILLA).reset_index(drop=True)

    resumen = {}
    alertas = 0
    for i, fila in muestras.iterrows():
        # ETAPA 1: Adquisicion (una muestra)
        cruda = fila.to_dict()
        # ETAPA 2: Preprocesado
        sensores = preprocesar(cruda)
        if sensores is None:
            print(f"[DESCARTADA] fila {i}: muestra invalida (vacios o formato)")
            continue
        # ETAPA 3+4: Inferencia y salida accionable
        r = inferir_y_accionar(modelo, sensores, contador=i + 1)
        imprimir_salida(r)
        resumen[r["estado"]] = resumen.get(r["estado"], 0) + 1
        if r["nivel"] in ("AVISO", "ALERTA"):
            alertas += 1
        if pausa:
            time.sleep(pausa)

    print("-" * 70)
    print(f"RESUMEN: {sum(resumen.values())} muestras procesadas | {alertas} avisos/alertas generados")
    print("Conteo por estado clasificado:", dict(resumen))
    print("=" * 70)


# ------------------------------------------------------------
# MODO MQTT - inferencia en vivo desde el robot
# ------------------------------------------------------------
def modo_mqtt(modelo, broker):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("ERROR: falta paho-mqtt. Instala con: pip install paho-mqtt")
        return

    print("\n" + "=" * 70)
    print(f" MODO MQTT-LIVE  (Adquisicion en vivo desde {broker}:{MQTT_PORT})")
    print(" Esperando telemetria del robot... (Ctrl+C para salir)")
    print("=" * 70)

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("[MQTT] Conectado. Suscrito a", MQTT_TOPIC)
            client.subscribe(MQTT_TOPIC)
        else:
            print("[MQTT] Error de conexion, rc =", rc)

    def on_message(client, userdata, msg):
        try:
            cruda = json.loads(msg.payload.decode())
        except json.JSONDecodeError:
            return
        sensores = preprocesar(cruda)          # ETAPA 2
        if sensores is None:
            return
        r = inferir_y_accionar(modelo, sensores, cruda.get("contador"))  # ETAPA 3+4
        imprimir_salida(r)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(broker, MQTT_PORT, 60)
        client.loop_forever()
    except Exception as e:
        print("[MQTT] No se pudo conectar:", e)


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Pipeline E2E del gemelo digital del seguidor de linea")
    ap.add_argument("--modo", choices=["csv", "mqtt"], default="csv",
                    help="csv = replay del dataset | mqtt = inferencia en vivo")
    ap.add_argument("--broker", default="192.168.137.1", help="IP del broker MQTT (modo mqtt)")
    ap.add_argument("--limite", type=int, default=25, help="Numero de muestras a procesar (modo csv)")
    args = ap.parse_args()

    print("############################################################")
    print("#  PIPELINE E2E - GEMELO DIGITAL ROBOT SEGUIDOR DE LINEA    #")
    print("#  Adquisicion -> Preprocesado -> Inferencia -> Salida      #")
    print("############################################################")

    modelo = obtener_modelo()   # ETAPA 0

    if args.modo == "csv":
        modo_csv(modelo, limite=args.limite)
    else:
        modo_mqtt(modelo, args.broker)
