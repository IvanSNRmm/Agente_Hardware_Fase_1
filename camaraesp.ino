#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
// #define USAR_LED Comentada para evitar uso del LED jeje
#ifdef USAR_LED
#define LED_FLASH 4  // GPIO4 - LED integrado de la ESP32-CAM
#endif

// Configuración de la RED
// const char* ssid = "INFINITUMB651";
// const char* password = "uXecP6cTTG";
const char* ssid = "BUAP_Estudiantes";
const char* password = "f85ac21de4";
// const char* ssid = "POCO M6 Pro";
// const char* password = "12345678";
WebServer server(80);

// Uso de LED para indicar conexión a WiFi (Encendido al estar conectado)
// Debido a potencia de este, se desactivo
#ifdef USAR_LED
void setupLED() {
  pinMode(LED_FLASH, OUTPUT);
  digitalWrite(LED_FLASH, LOW); // Iniciar apagado
  Serial.println(" LED Flash configurado en GPIO 4");
}

void controlarLED(bool encender) {
  digitalWrite(LED_FLASH, encender ? HIGH : LOW);
}
#endif

// Verificación de PSRAM
void verificarMemoria() {
  Serial.println("\n  DIAGNÓSTICO DE MEMORIA ESP32-CAM ");
  Serial.printf(" Memoria RAM libre: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("PSRAM disponible: %s\n", psramFound() ? " SI" : " NO");
  //Conversión de capacidad a MB
  if (psramFound()) {
    Serial.printf(" PSRAM total: %d bytes (%.1f MB)\n", ESP.getPsramSize(), ESP.getPsramSize() / 1024.0 / 1024.0);
    Serial.printf(" PSRAM libre: %d bytes (%.1f MB)\n", ESP.getFreePsram(), ESP.getFreePsram() / 1024.0 / 1024.0);
  } else {
    // En caso de no tener, bajar en exceso calidad de la imagen
    Serial.println("  No se detectó PSRAM - Usando memoria interna solamente");
  }
  
  // Información adicional de procesamiento
  Serial.printf(" Frecuencia CPU: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf(" Tamaño del sketch: %d bytes\n", ESP.getSketchSize());
  Serial.printf(" Flash libre: %d bytes\n", ESP.getFreeSketchSpace());
  Serial.println("==============================================\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(" Iniciando ESP32-CAM...");
  
  // Verificar presencia de PSRAM y capacidad total de memoria
  verificarMemoria();
  
  // Caso de querer usar LED
  #ifdef USAR_LED
  setupLED();
  #endif
  // Configuración de la ESP32 CAM (PINES,RELOJ, PERIFERICOS)
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Ajuste de calidad del stream 
  // Importante verificar esto correctamente, tamaño de frame y calidad
  // Dependiente de PSRAM 
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;  // 800x600
    config.jpeg_quality = 10;            // Mejor calidad posible (0-63, menor -> mejor)
    config.fb_count = 2;                 // Dos buffers para más fluidez (Mayor procesamiento) 
    Serial.println(" Configuración: "ALTA" calidad (con PSRAM)");
    Serial.println("   • Resolución: 800x600 (SVGA)");
    Serial.println("   • Calidad JPEG:" + String(config.jpeg_quality) + "/63");
    Serial.println("   • Buffers: 2");
  } else {
    // En caso de no tener PSRAM
    config.frame_size = FRAMESIZE_VGA;   // 640x480 - Menor detalle
    config.jpeg_quality = 20;            // Menor calidad
    config.fb_count = 1;                 // Un solo buffer
    Serial.println(" Configuración: "BAJA" calidad (sin PSRAM)");
    Serial.println("   • Resolución: 640x480 (VGA)");
    Serial.println("   • Calidad JPEG: 20/63");
    Serial.println("   • Buffers: 1");
  }
  
  // Inicializar cámara con operador &
  esp_err_t err = esp_camera_init(&config);
  // Si no se recibe ESP_OK eror de inicialización de camara (no detectada o no soportada)
  if (err != ESP_OK) {
    Serial.printf(" Error inicializando cámara: 0x%x\n", err);
    return;
  }
  Serial.println(" Cámara inicializada correctamente");
  
  // Conexión de Wifi pero con "Timeout"(intento de replica) para manejo de errores (evitar Bucles infinitos)
  Serial.println();
  Serial.print(" Conectando a WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  int intentos = 0;
  const int max_intentos = 30; // tiempo aproximado a 15seg para conexión
  #ifdef USAR_LED
  Serial.print("   ");
  // En caso de no estar conectado y no haber llegado a limite de intentos
  while (WiFi.status() != WL_CONNECTED && intentos < max_intentos) {
    delay(500);
    Serial.print("."); // Indica en Serial monitor el número de intentos de conexión 
    digitalWrite(LED_FLASH, !digitalRead(LED_FLASH)); // Parpadeo solo si LED activado
    intentos++;
  }
  #else
  // Caso de no definir USAR LED, para que se ejecute 
  Serial.print("   ");
  while (WiFi.status() != WL_CONNECTED && intentos < max_intentos) {
    delay(500);
    Serial.print("."); // Indica en Serial monitor el número de intentos de conexión 
    intentos++;
  }
  #endif
  // Verificar si se conectó
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n WiFi conectado");
    Serial.print(" Dirección IP: ");
    Serial.println(WiFi.localIP());
    
    // Encender LED solo si esta definido (no definido por exceso de Luz)
    #ifdef USAR_LED
    controlarLED(true); // LED encendido cuando hay conexión
    Serial.println(" LED encendido - Indicador de conexión WiFi");
    #endif
  } else {// Fallo total de intento de conexión 
    Serial.println("\n ERROR: No se pudo conectar al WiFi");
    Serial.println("   • Verifica que la red esté disponible");
    Serial.println("   • Verifica usuario y contraseña");
    Serial.println("   • Reinicia el ESP32-CAM");
    
    #ifdef USAR_LED
    controlarLED(false); // LED apagado en caso de error
    #endif
    
    // No continuar si no hay WiFi
    return;
  }
  
  // Configuración del servidor para ESP32 CAM
  server.on("/stream", HTTP_GET, handleStream);
  
  // Página de información
  server.on("/", HTTP_GET, []() {
    String html = "<html><head><title>ESP32-CAM Info</title>";
    html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>body{font-family:Arial; margin:20px; background:#f5f5f5;}";
    html += ".info{padding:15px; background:white; border-radius:10px; margin:10px 0;}";
    html += ".psram{color:" + String(psramFound() ? "green" : "red") + "; font-weight:bold;}</style></head><body>";
    html += "<h1>📷 ESP32-CAM - Información del Sistema</h1>";
    
    html += "<div class='info'><h3>🔍 Memoria</h3>";
    html += "<p><strong>PSRAM:</strong> <span class='psram'>" + String(psramFound() ? "DISPONIBLE " : "NO DISPONIBLE ") + "</span></p>";
    html += "<p><strong>RAM libre:</strong> " + String(ESP.getFreeHeap()) + " bytes</p>";
    if(psramFound()) {
      html += "<p><strong>PSRAM total:</strong> " + String(ESP.getPsramSize()) + " bytes</p>";
      html += "<p><strong>PSRAM libre:</strong> " + String(ESP.getFreePsram()) + " bytes</p>";
    }
    html += "</div>";
    
    html += "<div class='info'><h3> Configuración Cámara</h3>";
    html += "<p><strong>Resolución:</strong> " + String(psramFound() ? "800x600 (SVGA)" : "640x480 (VGA)") + "</p>";
    html += "<p><strong>Calidad JPEG:</strong> " + String(psramFound() ? "12/63" : "20/63") + "</p>";
    html += "<p><strong>Buffers:</strong> " + String(psramFound() ? "2" : "1") + "</p>";
    html += "</div>";
    
    html += "<div class='info'><h3> Streaming</h3>";
    html += "<p><a href='/stream' target='_blank'> Ver stream de video</a></p>";
    html += "</div>";
    
    html += "</body></html>";
    server.send(200, "text/html", html);
  });
  // Uso de puerto 80 debido a que es el puerto por defecto
  // Imprimir IP local para conexión con Python
  
  server.begin();
  Serial.println(" Servidor HTTP iniciado en puerto 80");
  Serial.println(" Página de info: http://" + WiFi.localIP().toString());
  Serial.println(" Stream: http://" + WiFi.localIP().toString() + "/stream");
}
// Implementa un servidor de video en streaming MJPEG para el ESP32-CAM
// MJPEG sucesión de frames JPEG

void handleStream() {
  // Manejo de solicitudes de clientes (Dispositivos queriendo conectar)
  WiFiClient client = server.client();
  
  // Encabezados MJPEG
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Access-Control-Allow-Origin: *");
  client.println();
  
  while (true) {
    camera_fb_t * fb = esp_camera_fb_get(); // captura el frame y verificación de exito
    if (!fb) {
      Serial.println(" Error capturando frame");
      break;
    }
    // Transmisión del frame  con sus encambezados
    client.println("--frame");
    client.println("Content-Type: image/jpeg");
    client.printf("Content-Length: %d\n\n", fb->len);
    client.write(fb->buf, fb->len);
    // Liberación memoria del frame
    esp_camera_fb_return(fb);
    // Pequeña pausa para fluidez (≈200 fps)
    delay(5);
  }
}
// Después de que setup se inicia correctamente para configuración de todo (en caso de WiFi fallar programa se termina)
// Conexión exitosa -> loop entra y escucha el puerto 80
// Llega solicitud -> se ejecuta función de arriba 
// handleStream llamada-> entra while-> se bloquea void loop-> se transmite video hasta error o desconexión
// Motivo del reincio de la camara al finalizar programa de python
void loop() {
  server.handleClient();
}