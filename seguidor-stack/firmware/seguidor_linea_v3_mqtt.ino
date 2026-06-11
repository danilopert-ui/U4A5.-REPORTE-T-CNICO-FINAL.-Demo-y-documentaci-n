/*
  ============================================================
  SEGUIDOR DE LINEA v3 · CONEXION MQTT (Fase 1 del gemelo digital)
  ESP32 + TB6612FNG + 4x QTR-1A · Arduino Core 3.x
  ============================================================

  CAMBIOS respecto a la v2:
   - WiFi en modo STA: el robot se UNE al hotspot de la laptop
     (ya no crea su propia red AP).
   - Cliente MQTT (PubSubClient): publica toda la telemetria en
     formato JSON al broker Mosquitto que corre en la laptop.
   - El panel web de PID se mantiene, ahora en la IP que asigna
     el hotspot (mirar el Monitor Serie al arrancar).

  --- CONFIGURACION DE RED (ajustada a tu hotspot) ---
     SSID:        RobotNet
     Password:    seguidor2026
     Broker MQTT: 192.168.137.1  (IP de la laptop en el hotspot)
     Puerto:      1883
     Topic:       seguidor/telemetria

  --- LIBRERIAS A INSTALAR ---
     - QTRSensors (Pololu)
     - PubSubClient (Nick O'Leary)   [Library Manager]
  ============================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <QTRSensors.h>
#include <Preferences.h>
#include <PubSubClient.h>     // <-- NUEVO: cliente MQTT

// ============================================================
// CONFIGURACION DE RED Y MQTT
// ============================================================
const char* WIFI_SSID = "RobotNet";
const char* WIFI_PASS = "seguidor2026";

const char* MQTT_BROKER = "192.168.137.1";   // IP de la laptop en el hotspot
const int   MQTT_PORT   = 1883;
const char* MQTT_TOPIC  = "seguidor/telemetria";
const char* MQTT_CLIENT_ID = "seguidor-esp32";

// Cada cuanto publicar telemetria por MQTT (ms)
const unsigned long INTERVALO_MQTT = 200;

WiFiClient espClient;
PubSubClient mqtt(espClient);
WebServer server(80);
Preferences prefs;

// ============================================================
// CONFIGURACION DE PINES
// ============================================================
const uint8_t NUM_SENSORES = 4;
const uint8_t sensorPins[NUM_SENSORES] = {36, 39, 34, 35};

const int STBY = 13;
const int AIN1 = 14;
const int AIN2 = 27;
const int PWMA = 26;
const int BIN1 = 25;
const int BIN2 = 33;
const int PWMB = 32;

// ============================================================
// MOTORES
// ============================================================
const int FREQ_PWM = 20000;
const int RES_PWM = 8;
const bool INVERTIR_A = true;
const bool INVERTIR_B = true;

// ============================================================
// CONTROL
// ============================================================
const int PERIODO_CONTROL_MS = 5;

volatile float Kp = 0.08;
volatile float Ki = 0.0;
volatile float Kd = 0.6;
volatile int VELOCIDAD_BASE = 100;
volatile int VELOCIDAD_MAX  = 200;
volatile bool LINEA_NEGRA   = true;
volatile int VELOCIDAD_RECTA = 180;
volatile int VELOCIDAD_CURVA = 90;
volatile float UMBRAL_CURVA  = 600.0;

const float INTEGRAL_MAX = 1000.0;
const uint16_t POSICION_OBJETIVO = 1500;

const unsigned long T_RECUP_FASE2 = 400;
const unsigned long T_RECUP_STOP  = 3000;
const int VEL_RECUPERACION = 120;

// ============================================================
// ESTADO
// ============================================================
enum Estado { DETENIDO, SIGUIENDO, CALIBRANDO };
volatile Estado estadoActual = DETENIDO;
volatile bool solicitarCalibracion = false;
volatile bool solicitarGuardado = false;
volatile bool calibrado = false;

QTRSensors qtr;
uint16_t lecturasSensores[NUM_SENSORES];

float errorAnterior = 0;
float integral = 0;
float ultimoError = 0;
bool lineaPerdida = false;
unsigned long tiempoLineaPerdida = 0;

// Telemetria compartida
volatile uint16_t t_posicion = 1500;
volatile float t_error = 0;
volatile float t_correccion = 0;
volatile int t_velIzq = 0;
volatile int t_velDer = 0;
volatile int t_velBaseActual = 0;
volatile uint16_t t_crudo[NUM_SENSORES] = {0,0,0,0};

// ============================================================
// MOTORES
// ============================================================
void motorA(int v) {
  if (INVERTIR_A) v = -v;
  v = constrain(v, -255, 255);
  if (v > 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); ledcWrite(PWMA, v); }
  else if (v < 0) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); ledcWrite(PWMA, -v); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); ledcWrite(PWMA, 0); }
}
void motorB(int v) {
  if (INVERTIR_B) v = -v;
  v = constrain(v, -255, 255);
  if (v > 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); ledcWrite(PWMB, v); }
  else if (v < 0) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); ledcWrite(PWMB, -v); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); ledcWrite(PWMB, 0); }
}
void detener() { motorA(0); motorB(0); }

void leerSensoresCrudos() {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) t_crudo[i] = analogRead(sensorPins[i]);
}

// ============================================================
// NOMBRE DEL ESTADO (texto, para MQTT/IA)
// ============================================================
const char* nombreEstado() {
  if (estadoActual == CALIBRANDO) return "calibrando";
  if (estadoActual == DETENIDO)   return "detenido";
  if (lineaPerdida)               return "recuperacion";
  // Si esta siguiendo y ve la linea, clasificar por el error
  float e = t_error;
  if (e > 400)  return "desviado_der";
  if (e < -400) return "desviado_izq";
  return "centrado";
}

// ============================================================
// FLASH (guardar/cargar parametros)
// ============================================================
void guardarParametros() {
  prefs.begin("seguidor", false);
  prefs.putFloat("kp", Kp); prefs.putFloat("ki", Ki); prefs.putFloat("kd", Kd);
  prefs.putInt("vbase", VELOCIDAD_BASE); prefs.putInt("vmax", VELOCIDAD_MAX);
  prefs.putInt("vrecta", VELOCIDAD_RECTA); prefs.putInt("vcurva", VELOCIDAD_CURVA);
  prefs.putFloat("ucurva", UMBRAL_CURVA); prefs.putBool("lnegra", LINEA_NEGRA);
  prefs.end();
  Serial.println("[FLASH] Guardado");
}
void cargarParametros() {
  prefs.begin("seguidor", true);
  Kp = prefs.getFloat("kp", Kp); Ki = prefs.getFloat("ki", Ki); Kd = prefs.getFloat("kd", Kd);
  VELOCIDAD_BASE = prefs.getInt("vbase", VELOCIDAD_BASE);
  VELOCIDAD_MAX = prefs.getInt("vmax", VELOCIDAD_MAX);
  VELOCIDAD_RECTA = prefs.getInt("vrecta", VELOCIDAD_RECTA);
  VELOCIDAD_CURVA = prefs.getInt("vcurva", VELOCIDAD_CURVA);
  UMBRAL_CURVA = prefs.getFloat("ucurva", UMBRAL_CURVA);
  LINEA_NEGRA = prefs.getBool("lnegra", LINEA_NEGRA);
  prefs.end();
  Serial.println("[FLASH] Cargado");
}

// ============================================================
// CALIBRACION
// ============================================================
void calibrar() {
  estadoActual = CALIBRANDO;
  Serial.println("[CALIB] Iniciando...");
  motorA(80); motorB(-80);
  for (uint16_t i = 0; i < 250; i++) { qtr.calibrate(); delay(1); }
  detener();
  calibrado = true;
  estadoActual = DETENIDO;
  Serial.println("[CALIB] Completada");
}

// ============================================================
// PID
// ============================================================
float calcularPID(float error, float dt) {
  float P = Kp * error;
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  float I = Ki * integral;
  float D = Kd * (error - errorAnterior) / dt;
  errorAnterior = error;
  return P + I + D;
}

// ============================================================
// UN PASO DE SEGUIMIENTO
// ============================================================
void pasoSeguimiento(float dt) {
  uint16_t posicion = LINEA_NEGRA ? qtr.readLineBlack(lecturasSensores)
                                  : qtr.readLineWhite(lecturasSensores);
  bool veLinea = false;
  for (uint8_t i = 0; i < NUM_SENSORES; i++)
    if (lecturasSensores[i] > 200) { veLinea = true; break; }

  if (!veLinea) {
    if (!lineaPerdida) { lineaPerdida = true; tiempoLineaPerdida = millis(); }
    unsigned long perdidoHace = millis() - tiempoLineaPerdida;
    if (perdidoHace > T_RECUP_STOP) {
      detener(); estadoActual = DETENIDO;
      Serial.println("[RECUP] Detenido por seguridad");
      return;
    }
    int dir = (ultimoError >= 0) ? 1 : -1;
    int velGiro = (perdidoHace < T_RECUP_FASE2) ? VEL_RECUPERACION : VEL_RECUPERACION + 30;
    motorA(dir * velGiro); motorB(-dir * velGiro);
    t_posicion = posicion; t_error = ultimoError; t_correccion = 0;
    t_velIzq = dir * velGiro; t_velDer = -dir * velGiro; t_velBaseActual = 0;
    return;
  }

  lineaPerdida = false;
  float error = (float)posicion - (float)POSICION_OBJETIVO;
  ultimoError = error;
  float correccion = calcularPID(error, dt);

  float factor = constrain(fabs(error) / UMBRAL_CURVA, 0.0, 1.0);
  int velBase = VELOCIDAD_RECTA - (int)((VELOCIDAD_RECTA - VELOCIDAD_CURVA) * factor);

  int velIzq = constrain(velBase + (int)correccion, -VELOCIDAD_MAX, VELOCIDAD_MAX);
  int velDer = constrain(velBase - (int)correccion, -VELOCIDAD_MAX, VELOCIDAD_MAX);
  motorA(velIzq); motorB(velDer);

  t_posicion = posicion; t_error = error; t_correccion = correccion;
  t_velIzq = velIzq; t_velDer = velDer; t_velBaseActual = velBase;
}

// ============================================================
// TAREA DE CONTROL (nucleo 1)
// ============================================================
void tareaControl(void* p) {
  TickType_t ultimoDespertar = xTaskGetTickCount();
  const TickType_t periodo = pdMS_TO_TICKS(PERIODO_CONTROL_MS);
  const float dt = PERIODO_CONTROL_MS / 1000.0;
  for (;;) {
    leerSensoresCrudos();
    if (solicitarCalibracion) { solicitarCalibracion = false; calibrar(); }
    if (solicitarGuardado) { solicitarGuardado = false; guardarParametros(); }
    if (estadoActual == SIGUIENDO) pasoSeguimiento(dt);
    vTaskDelayUntil(&ultimoDespertar, periodo);
  }
}

// ============================================================
// PUBLICAR TELEMETRIA POR MQTT (JSON)
// ============================================================
void publicarMQTT() {
  // Construir el JSON con todos los datos del instante actual
  char payload[320];
  snprintf(payload, sizeof(payload),
    "{\"estado\":\"%s\",\"pos\":%u,\"error\":%d,\"correccion\":%d,"
    "\"velIzq\":%d,\"velDer\":%d,\"velBase\":%d,\"perdida\":%s,"
    "\"s\":[%u,%u,%u,%u],\"crudo\":[%u,%u,%u,%u]}",
    nombreEstado(), t_posicion, (int)t_error, (int)t_correccion,
    t_velIzq, t_velDer, t_velBaseActual, lineaPerdida ? "true" : "false",
    lecturasSensores[0], lecturasSensores[1], lecturasSensores[2], lecturasSensores[3],
    t_crudo[0], t_crudo[1], t_crudo[2], t_crudo[3]
  );
  mqtt.publish(MQTT_TOPIC, payload);
}

// Reconectar a MQTT si se cae
void reconectarMQTT() {
  if (mqtt.connected()) return;
  if (mqtt.connect(MQTT_CLIENT_ID)) {
    Serial.println("[MQTT] Conectado al broker");
  } else {
    Serial.print("[MQTT] Fallo, rc="); Serial.println(mqtt.state());
  }
}

// ============================================================
// PAGINA WEB (igual que v2, omitida aqui por brevedad de logica;
// se sirve el panel de PID con los mismos endpoints)
// ============================================================
const char PAGINA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="es"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Seguidor v3</title>
<style>
 body{font-family:system-ui,sans-serif;background:#0f1419;color:#e6e6e6;padding:16px;max-width:480px;margin:0 auto}
 h1{font-size:1.2rem;text-align:center;color:#4fc3f7}
 .estado{text-align:center;padding:10px;border-radius:10px;font-weight:bold;margin-bottom:14px}
 .detenido{background:#37474f}.siguiendo{background:#2e7d32}.calibrando{background:#f57f17}
 .mqtt{text-align:center;font-size:.8rem;padding:6px;border-radius:8px;margin-bottom:14px}
 .mqtt.ok{background:#1b3a2b;color:#66bb6a}.mqtt.no{background:#3a1f1f;color:#ff5252}
 .botones{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:12px}
 button{border:none;border-radius:10px;padding:16px;font-size:1rem;font-weight:bold;color:#fff}
 .bi{background:#2e7d32}.bc{background:#f57f17}.bp{background:#c62828;grid-column:1/-1;padding:20px}
 .bg{background:#6a1b9a;grid-column:1/-1}
 .param{background:#1a2330;border-radius:10px;padding:12px;margin-bottom:10px}
 .ph{display:flex;justify-content:space-between;margin-bottom:6px}
 .pn{font-weight:bold;color:#4fc3f7}.pv{font-family:monospace;color:#fff}
 input[type=range]{width:100%;height:34px}
 .tele{background:#1a2330;border-radius:10px;padding:12px;font-family:monospace;font-size:.85rem;line-height:1.6}
 .tr{display:flex;justify-content:space-between}.tl{color:#90a4ae}.tv{color:#4fc3f7;font-weight:bold}
 .st{font-size:.8rem;color:#607d8b;text-transform:uppercase;margin:14px 0 6px}
</style></head><body>
<h1>🤖 Seguidor v3 (MQTT)</h1>
<div id="estado" class="estado detenido">DETENIDO</div>
<div id="mqtt" class="mqtt no">MQTT: ...</div>
<div class="botones">
 <button class="bi" onclick="cmd('iniciar')">▶ INICIAR</button>
 <button class="bc" onclick="cmd('calibrar')">⚙ CALIBRAR</button>
 <button class="bp" onclick="cmd('parar')">⬛ PARAR</button>
 <button class="bg" onclick="guardar()">💾 GUARDAR EN FLASH</button>
</div>
<div class="st">PID</div>
<div class="param"><div class="ph"><span class="pn">Kp</span><span class="pv" id="vkp">0.080</span></div>
 <input type="range" id="kp" min="0" max="0.5" step="0.005" value="0.08" oninput="up('kp',this.value)"></div>
<div class="param"><div class="ph"><span class="pn">Ki</span><span class="pv" id="vki">0.000</span></div>
 <input type="range" id="ki" min="0" max="0.1" step="0.001" value="0" oninput="up('ki',this.value)"></div>
<div class="param"><div class="ph"><span class="pn">Kd</span><span class="pv" id="vkd">0.600</span></div>
 <input type="range" id="kd" min="0" max="3" step="0.01" value="0.6" oninput="up('kd',this.value)"></div>
<div class="st">Velocidad</div>
<div class="param"><div class="ph"><span class="pn">Vel recta</span><span class="pv" id="vvr">180</span></div>
 <input type="range" id="vrecta" min="0" max="255" step="5" value="180" oninput="up('vrecta',this.value)"></div>
<div class="param"><div class="ph"><span class="pn">Vel curva</span><span class="pv" id="vvc">90</span></div>
 <input type="range" id="vcurva" min="0" max="255" step="5" value="90" oninput="up('vcurva',this.value)"></div>
<div class="param"><div class="ph"><span class="pn">Umbral curva</span><span class="pv" id="vuc">600</span></div>
 <input type="range" id="ucurva" min="100" max="1500" step="50" value="600" oninput="up('ucurva',this.value)"></div>
<div class="st">Telemetria</div>
<div class="tele">
 <div class="tr"><span class="tl">Estado</span><span class="tv" id="t-est">--</span></div>
 <div class="tr"><span class="tl">Posicion</span><span class="tv" id="t-pos">--</span></div>
 <div class="tr"><span class="tl">Error</span><span class="tv" id="t-err">--</span></div>
 <div class="tr"><span class="tl">Vel Izq</span><span class="tv" id="t-vi">--</span></div>
 <div class="tr"><span class="tl">Vel Der</span><span class="tv" id="t-vd">--</span></div>
</div>
<script>
 function cmd(a){fetch('/cmd?a='+a)}
 function guardar(){fetch('/guardar')}
 function up(n,v){fetch('/param?n='+n+'&v='+v);
  var m={kp:'vkp',ki:'vki',kd:'vkd',vrecta:'vvr',vcurva:'vvc',ucurva:'vuc'};
  var el=document.getElementById(m[n]);
  if(['kp','ki','kd'].includes(n))el.textContent=parseFloat(v).toFixed(3);else el.textContent=v;}
 setInterval(function(){fetch('/tele').then(r=>r.json()).then(d=>{
  var e=document.getElementById('estado');e.textContent=d.estado.toUpperCase();e.className='estado '+(d.estado=='siguiendo'?'siguiendo':d.estado=='calibrando'?'calibrando':'detenido');
  var m=document.getElementById('mqtt');if(d.mqtt){m.className='mqtt ok';m.textContent='MQTT: conectado';}else{m.className='mqtt no';m.textContent='MQTT: desconectado';}
  document.getElementById('t-est').textContent=d.estadoTxt;
  document.getElementById('t-pos').textContent=d.pos;
  document.getElementById('t-err').textContent=d.err;
  document.getElementById('t-vi').textContent=d.vi;
  document.getElementById('t-vd').textContent=d.vd;
 }).catch(e=>{})},250);
</script></body></html>
)rawliteral";

// ============================================================
// HANDLERS WEB
// ============================================================
void handleRoot() { server.send_P(200, "text/html", PAGINA_HTML); }
void handleCmd() {
  String a = server.arg("a");
  if (a == "iniciar") {
    if (calibrado) { integral=0; errorAnterior=0; lineaPerdida=false; estadoActual=SIGUIENDO; }
  } else if (a == "parar") { estadoActual = DETENIDO; detener(); }
  else if (a == "calibrar") { solicitarCalibracion = true; }
  server.send(200, "text/plain", "ok");
}
void handleParam() {
  String n = server.arg("n"); String v = server.arg("v");
  if (n=="kp") Kp=v.toFloat(); else if (n=="ki") Ki=v.toFloat(); else if (n=="kd") Kd=v.toFloat();
  else if (n=="vrecta") VELOCIDAD_RECTA=v.toInt(); else if (n=="vcurva") VELOCIDAD_CURVA=v.toInt();
  else if (n=="ucurva") UMBRAL_CURVA=v.toFloat(); else if (n=="vmax") VELOCIDAD_MAX=v.toInt();
  else if (n=="linea") LINEA_NEGRA=(v.toInt()==1);
  server.send(200, "text/plain", "ok");
}
void handleGuardar() { solicitarGuardado = true; server.send(200, "text/plain", "ok"); }
void handleTele() {
  String est = (estadoActual==DETENIDO)?"detenido":(estadoActual==SIGUIENDO)?"siguiendo":"calibrando";
  String json = "{";
  json += "\"estado\":\"" + est + "\",";
  json += "\"estadoTxt\":\"" + String(nombreEstado()) + "\",";
  json += "\"mqtt\":" + String(mqtt.connected() ? "true" : "false") + ",";
  json += "\"pos\":" + String(t_posicion) + ",";
  json += "\"err\":" + String((int)t_error) + ",";
  json += "\"vi\":" + String(t_velIzq) + ",";
  json += "\"vd\":" + String(t_velDer);
  json += "}";
  server.send(200, "application/json", json);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, FREQ_PWM, RES_PWM);
  ledcAttach(PWMB, FREQ_PWM, RES_PWM);
  detener();
  digitalWrite(STBY, HIGH);
  analogReadResolution(12);

  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, NUM_SENSORES);
  qtr.setSamplesPerSensor(4);

  cargarParametros();

  // ----- WiFi en modo STA (se une al hotspot) -----
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando a WiFi '"); Serial.print(WIFI_SSID); Serial.print("'");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("====================================================");
    Serial.print("WiFi conectado. IP del robot: ");
    Serial.println(WiFi.localIP());      // <-- ABRE EL PANEL WEB EN ESTA IP
    Serial.println("====================================================");
  } else {
    Serial.println("[WiFi] NO se pudo conectar. Revisa SSID/PASS y que el hotspot este encendido.");
  }

  // ----- MQTT -----
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  reconectarMQTT();

  // ----- Servidor web -----
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/param", handleParam);
  server.on("/guardar", handleGuardar);
  server.on("/tele", handleTele);
  server.begin();
  Serial.println("Servidor web iniciado.");

  // ----- Tarea de control en nucleo 1 -----
  xTaskCreatePinnedToCore(tareaControl, "Control", 8192, NULL, 2, NULL, 1);
  Serial.println("Tarea de control en nucleo 1. Sistema listo.");
}

// ============================================================
// LOOP (nucleo 0/1: web + MQTT)
// ============================================================
void loop() {
  server.handleClient();

  // Mantener viva la conexion MQTT
  if (!mqtt.connected()) {
    static unsigned long ultimoIntento = 0;
    if (millis() - ultimoIntento > 2000) {   // reintenta cada 2s
      ultimoIntento = millis();
      reconectarMQTT();
    }
  } else {
    mqtt.loop();
    // Publicar telemetria periodicamente
    static unsigned long ultimoPub = 0;
    if (millis() - ultimoPub > INTERVALO_MQTT) {
      ultimoPub = millis();
      publicarMQTT();
    }
  }

  delay(1);
}
