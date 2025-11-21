#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include "driver/ledc.h"

// Configuración de Pines de los Puente H
// Motores de ATRÁS 
#define IN1 14    // d14
#define IN2 27    // d27  
#define ENA 26    // d26 (PWM)
#define IN3 25    // d32 
#define IN4 33    // d33
#define ENB 32    // d25 (PWM)
// Motores de ADELANTE 
#define INT1 12   // d12
#define INT2 13   // d13  
#define ENTA 23   // d23 (PWM)
#define INT3 4    // d4
#define INT4 5    // d5
#define ENTB 18   // d18 (PWM)
// Configuración de LED en GPIO 2
#define LED_INTEGRADO 2  // Integrado en placa
// Configuración del PWM
const int freq = 5000;
const int resolution = 8;
const int VELOCIDAD_FIJA = 100 ; // Velocidad para mantener video estable
// Canales PWM para configuración
const int canalENA = 0;
const int canalENB = 1;
const int canalENTA = 2;
const int canalENTB = 3;
// Configuración de las Redes WiFi
//const char* ssid = "POCO M6 Pro";
//const char* password = "12345678";
//const char* ssid = "Ivan";
//const char* password = "12345678";
//const char* ssid = "A52";
//const char* password = "12345abcde";
const char* ssid = "A56 de Reyna";
const char* password = "54321abcd";
WebServer server(80);
String estado_actual = "DETENIDO";
// Configuración de los motores
void setupMotores() {
  Serial.println("Configurando pines de motores...");
  
  // Configurar pines de dirección como OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(INT1, OUTPUT);
  pinMode(INT2, OUTPUT);
  pinMode(INT3, OUTPUT);
  pinMode(INT4, OUTPUT);
  
  // Configurar PWM para pines ENA/ENB
  ledcSetup(canalENA, freq, resolution);
  ledcSetup(canalENB, freq, resolution);
  ledcSetup(canalENTA, freq, resolution);
  ledcSetup(canalENTB, freq, resolution);
  // Asignar canales PWM a los pines ENA/ENB
  ledcAttachPin(ENA, canalENA);
  ledcAttachPin(ENB, canalENB);
  ledcAttachPin(ENTA, canalENTA);
  ledcAttachPin(ENTB, canalENTB);
  // Inicialmente apagar todos los motores
  detenerMotores();
  Serial.println("Motores configurados correctamente");
}
// Función para configurar LED de conexión
void setupLED() {
  pinMode(LED_INTEGRADO, OUTPUT);
  digitalWrite(LED_INTEGRADO, LOW); // Iniciar con LED apagado 
  Serial.println("LED integrado configurado en pin 2");
}
// Función de control de LED de conexión
void controlarLED(bool encender) {
  if (encender) {
    digitalWrite(LED_INTEGRADO, HIGH);
    Serial.println(" LED encendido - WiFi conectado");
  } else {
    digitalWrite(LED_INTEGRADO, LOW);
    Serial.println(" LED apagado - WiFi desconectado");
  }
}
// Configuración de la red WiFi
void setupWiFi() {
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  // Asegurar que el LED esté apagado al iniciar conexión
  controlarLED(false);
  WiFi.begin(ssid, password); 
  int intentos = 0;
  // Uso de "Timeout" para evitar errores en caso de no poder conectarse
  while (WiFi.status() != WL_CONNECTED && intentos < 20) { // 
    delay(500);
    Serial.print(".");
    // Parpadeo mientras intenta hacer conexión
    digitalWrite(LED_INTEGRADO, !digitalRead(LED_INTEGRADO));
    intentos++;
  }
  // Conexión exitosa y generación de IP local
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi conectado");
    Serial.print("Dirección IP: ");
    Serial.println(WiFi.localIP());
    // Encender LED permanentemente cuando se conecta
    controlarLED(true)
  } else {
    Serial.println("Error: No se pudo conectar al WiFi");
    controlarLED(false); // Asegurar que esté apagado
  }
}
// Configuración del servidor
void setupServer() {
  // Recibir los comandos de control con POST
  server.on("/control", HTTP_POST, manejarComando);
  // Devolver estado actual del robot con GET
  server.on("/estado", HTTP_GET, []() {
    String wifi_status = (WiFi.status() == WL_CONNECTED) ? "CONECTADO" : "DESCONECTADO";
    String respuesta = "{\"estado\":\"" + estado_actual + "\",\"velocidad\":" + String(VELOCIDAD_FIJA) + ",\"wifi\":\"" + wifi_status + "\"}";
    server.send(200, "application/json", respuesta);
  });
  // Pagina WEB con pequeña interfaz
  server.on("/", HTTP_GET, []() {
    String wifi_status = (WiFi.status() == WL_CONNECTED) ? "CONECTADO" : "DESCONECTADO";
    String wifi_ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "No disponible";
    String led_status = (digitalRead(LED_INTEGRADO) == HIGH) ? " ENCENDIDO" : " APAGADO";
    // Estructura de la página
    String html = "<html><head><title>Control Robot</title>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>";
    html += "body {font-family: Arial; margin: 40px; text-align: center; background-color: #f5f5f5;}";
    html += "button {padding: 20px 30px; margin: 15px; font-size: 18px; cursor: pointer; border: none; border-radius: 10px; transition: all 0.3s;}";
    html += ".avanzar {background-color: #4CAF50; color: white;}";
    html += ".avanzar:hover {background-color: #45a049; transform: scale(1.05);}";
    html += ".detener {background-color: #f44336; color: white;}";
    html += ".detener:hover {background-color: #da190b; transform: scale(1.05);}";
    html += ".estado {padding: 20px; margin: 20px; border-radius: 10px; background-color: white; box-shadow: 0 2px 5px rgba(0,0,0,0.1);}";
    html += ".wifi-status {padding: 15px; margin: 15px; border-radius: 8px; background-color: #e8f5e8;}";
    html += ".led-status {padding: 10px; margin: 10px; border-radius: 5px; background-color: #fff3cd;}";
    html += ".conectado {color: #155724; background-color: #d4edda;}";
    html += ".desconectado {color: #721c24; background-color: #f8d7da;}";
    html += "</style>";
    html += "</head><body>";
    html += "<h1> Control del Robot</h1>";
    html += "<h3>Velocidad fija: 90 PWM</h3>";
    // Estado del WiFi
    html += "<div class='wifi-status " + String((WiFi.status() == WL_CONNECTED) ? "conectado" : "desconectado") + "'>";
    html += "<h3> WiFi: " + wifi_status + "</h3>";
    html += "<p>IP: " + wifi_ip + "</p>";
    html += "</div>";
    // Estado del LED
    html += "<div class='led-status'>";
    html += "<h3> LED Integrado: " + led_status + "</h3>";
    html += "<p>El LED se enciende automáticamente cuando hay conexión WiFi</p>";
    html += "</div>";
    // Estado del robot
    html += "<div class='estado'>";
    html += "<h2>Estado del Robot: " + estado_actual + "</h2>";
    html += "<p>Velocidad: " + String(VELOCIDAD_FIJA) + "/255 (90 RPM)</p>";
    html += "</div>";
    // Botones de control (No se usan, solo eran para pruebas)
    html += "<div>";
    html += "<button class='avanzar' onclick=\"enviarComando('AVANZAR')\">";
    html += " AVANZAR";
    html += "</button>";
    // Botón detener
    html += "<button class='detener' onclick=\"enviarComando('DETENER')\">";
    html += " DETENER";
    html += "</button>";
    html += "</div>";
    html += "<br><hr>";
    html += "<p><strong>Configuración:</strong> Velocidad fija a 90 RPM</p>";
    html += "<p><strong>LED:</strong> Se enciende cuando hay conexión WiFi estable</p>";
    html += "<script>";
    html += "function enviarComando(comando) {";
    html += "  fetch('/control', {";
    html += "    method: 'POST',";
    html += "    headers: {'Content-Type': 'application/x-www-form-urlencoded'},";
    html += "    body: 'comando=' + comando";
    html += "  }).then(r => r.json()).then(data => {";
    html += "    alert('Comando ejecutado: ' + data.estado);";
    html += "    location.reload();";
    html += "  }).catch(err => {";
    html += "    alert('Error: ' + err);";
    html += "  });";
    html += "}";
    
    // Actualizar estado cada 5 segundos
    html += "setInterval(() => {";
    html += "  fetch('/estado').then(r => r.json()).then(data => {";
    html += "    if(data.wifi === 'DESCONECTADO') {";
    html += "      document.querySelector('.wifi-status').className = 'wifi-status desconectado';";
    html += "      document.querySelector('.wifi-status h3').innerHTML = '📶 WiFi: DESCONECTADO';";
    html += "    }";
    html += "  });";
    html += "}, 5000);";
    html += "</script>";
    html += "</body></html>";
    // Enviar página completa al cliente 
    server.send(200, "text/html", html);
  });
  // Iniciar el servidor
  server.begin();
  Serial.println("Servidor HTTP iniciado en puerto 80");
}

// Función del control de movimiento de los motores (Detener)
void detenerMotores() {
  Serial.println("Deteniendo todos los motores...");
  // Apagar pines de dirección
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(INT1, LOW);
  digitalWrite(INT2, LOW);
  digitalWrite(INT3, LOW);
  digitalWrite(INT4, LOW);
  // Apagar PWM
  ledcWrite(canalENA, 0);
  ledcWrite(canalENB, 0);
  ledcWrite(canalENTA, 0);
  ledcWrite(canalENTB, 0);
  estado_actual = "DETENIDO";
}
// Función del control de movimiento de los motores (Avanzar)
void avanzarMotores() {
  Serial.println("Avanzando todos los motores a 90 RPM...");
  // Configurar dirección para AVANZAR
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(INT1, HIGH);
  digitalWrite(INT2, LOW);
  digitalWrite(INT3, HIGH);
  digitalWrite(INT4, LOW);
  // Aplicar velocidad fija PWM a todos los motores
  ledcWrite(canalENA, VELOCIDAD_FIJA);
  ledcWrite(canalENB, VELOCIDAD_FIJA);
  ledcWrite(canalENTA, VELOCIDAD_FIJA);
  ledcWrite(canalENTB, VELOCIDAD_FIJA);
  estado_actual = "AVANZANDO";
}

// Manejo de peticiones
void manejarComando() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Método no permitido");
    return;
  }
  String comando = server.arg("comando");
  Serial.printf("Comando recibido: %s\n", comando.c_str());
  if (comando == "DETENER") {
    detenerMotores();
  } else if (comando == "AVANZAR") {
    avanzarMotores();
  } else {
    server.send(400, "application/json", "{\"error\":\"Comando no válido. Solo AVANZAR o DETENER\"}");
    return;
  }
  String respuesta = "{\"status\":\"ok\",\"estado\":\"" + estado_actual + "\",\"velocidad\":" + String(VELOCIDAD_FIJA) + "}";
  server.send(200, "application/json", respuesta);
  Serial.println("Comando ejecutado correctamente");
}

// En caso de perder conexión, poder volver a intentar reconectar
void verificarConexionWiFi() {
  static unsigned long ultimaVerificacion = 0;
  const unsigned long intervaloVerificacion = 10000; // Verificar cada 10 segundos
  if (millis() - ultimaVerificacion >= intervaloVerificacion) {
    ultimaVerificacion = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("⚠  WiFi desconectado. Intentando reconectar...");
      controlarLED(false);
      WiFi.disconnect();
      WiFi.reconnect();
      // Esperar un poco y verificar si se reconectó
      delay(5000);
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" WiFi reconectado");
        controlarLED(true);
      }
    }
  }
}

// ==================== SETUP Y LOOP PRINCIPAL ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando sistema de control del robot...");
  Serial.println("Velocidad fija: 90 RPM");
  Serial.println("===============================");
  setupLED();      // Configurar LED primero
  setupMotores();
  setupWiFi();     // WiFi controla el LED automáticamente
  setupServer();
  Serial.println("Sistema listo. Esperando comandos...");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Accede desde: http://" + WiFi.localIP().toString());
    Serial.println(" LED integrado: ENCENDIDO (WiFi conectado)");
  } else {
    Serial.println(" LED integrado: APAGADO (WiFi desconectado)");
  }
}
void loop() {
  server.handleClient();
  verificarConexionWiFi(); // Verificar conexión periódicamente
  delay(5);
}