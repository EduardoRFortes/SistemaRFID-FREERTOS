#include <WiFi.h>
#include <WiFiClientSecure.h> 
#include <PubSubClient.h>     
#include <ArduinoJson.h>
#include "RFIDR200.h" 
#include <sys/time.h>             
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <ESPping.h>


// ====================================================================
// ======= HEADERS DO FREERTOS PARA ARDUINO IDE ========
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
// ====================================================================

// Protótipos
void TaskConectarWiFi(void *pvParameters);
void TaskConectarMQTT(void *pvParameters);
void TaskPublicarMQTT(void *pvParameters);

// ======= DECLARAÇÕES DE OBJETOS E VARIÁVEIS GLOBAIS =================

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// OBJETOS DA BIBLIOTECA ROBUSTA (RFIDR200)
HardwareSerial MySerial(2);
RFIDR200 rfidReader(MySerial, 115200);

Preferences preferences;

SemaphoreHandle_t wifiConectadoSemaphore;
SemaphoreHandle_t mqttConectadoSemaphore;
QueueHandle_t rfidDataQueue; 

// CONFIGURAÇÕES DE REDE E MQTT (LIDAS DO NVS)
String NVS_SSID;
String NVS_SENHA;
String NVS_BROKER_MQTT;

String CLIENT_CERT_PROGMEM;
String CLIENT_KEY_PROGMEM;
String MQTT_CA_CERT_PROGMEM;

// OUTRAS CONSTANTES GLOBAIS
const int PORTA_MQTT = 8883;
const char* CLIENT_ID = "portal_ESP_02";
const char* TOPICO_MQTT = "/rfid/leituras";

// DEFININDO O INTERVALO MÍNIMO ESTÁVEL DE LEITURA (200ms)
const unsigned long T_INTERVALO_LEITURA_MS = 200; 

const char* ntpServer = "172.16.0.1";
const long gmtOffset_sec = -3 * 3600;
const int daylightOffset_sec = 0;

// Configuração do Watchdog (20 segundos)
#define WDT_TIMEOUT 20

// ====================================================================

// ======= FUNÇÃO AUXILIAR DE PUBLICAÇÃO MQTT (Otimizada) ========
// Alterado para receber const char* para evitar criação de Strings desnecessárias na entrada
void publicarMQTT(const char* epc) {
  if (!mqttClient.connected()) {
    Serial.println("PublicarMQTT: Conexão perdida. Tentando reconectar...");
    if(!mqttClient.connect(CLIENT_ID)) {
       return;
    }
  }

  int epochValue = time(nullptr);
  Serial.println(epochValue);
  if (mqttClient.connected()) {
      JsonDocument doc;
      doc["epc"] = epc;
      doc["mqttId"] = CLIENT_ID; 
      doc["timestamp"] = epochValue;

      String jsonString;
      serializeJson(doc, jsonString);

      if (mqttClient.publish(TOPICO_MQTT, jsonString.c_str())) {
        // Sucesso
      } else {
        Serial.println("Falha ao publicar via MQTT.");
      }
  }
}

// ======= TAREFA DE INICIALIZAÇÃO E RECONEXÃO WI-FI ========
void TaskConectarWiFi(void *pvParameters) {
  Serial.print("Task ConectarWiFi: Iniciando conexão Wi-Fi...");
  IPAddress primaryDNS(172, 16, 0, 1);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, primaryDNS, primaryDNS);
  WiFi.begin(NVS_SSID.c_str(), NVS_SENHA.c_str()); 

  Serial.println("Servidor DNS: ");
  Serial.println(WiFi.dnsIP());

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }

  Serial.println("\nTask ConectarWiFi: Wi-Fi conectado! IP: " + WiFi.localIP().toString());
  
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;

  Serial.println("Solicitando hora ao servidor interno (172.16.0.1)...");
  
  if (!getLocalTime(&timeinfo, 5000)) {
    Serial.println("ERRO: O servidor 172.16.0.1 não respondeu ao NTP.");
  } else {
    Serial.println(&timeinfo, "SUCESSO NTP INTERNO: %A, %B %d %Y %H:%M:%S");
  }

  xSemaphoreGive(wifiConectadoSemaphore); 
  Serial.println("Task ConectarWiFi: Concluída e se auto-deletando.");
  vTaskDelete(NULL);
}

// ======= TAREFA DE CONEXÃO E RECONEXÃO MQTT ========
void TaskConectarMQTT(void *pvParameters) {
  Serial.println("Task ConectarMQTT: Aguardando Wi-Fi...");
  xSemaphoreTake(wifiConectadoSemaphore, portMAX_DELAY); 
  
  const char* ca_cert = MQTT_CA_CERT_PROGMEM.c_str();
  const char* client_cert = CLIENT_CERT_PROGMEM.c_str();
  const char* client_key = CLIENT_KEY_PROGMEM.c_str();

  espClient.setCACert(ca_cert);
  espClient.setCertificate(client_cert);
  espClient.setPrivateKey(client_key);
  
  mqttClient.setServer(NVS_BROKER_MQTT.c_str(), PORTA_MQTT); 

  while (true) {
    if (!mqttClient.connected()) {
      Serial.print("Task ConectarMQTT: Conectando...");
      String currentClientId = String(CLIENT_ID) + "_" + String(esp_random(), HEX); 
      
      if (mqttClient.connect(currentClientId.c_str())) {
        Serial.println("\n[MQTT] Conectado!");
        xSemaphoreGive(mqttConectadoSemaphore); 
      } else {
        Serial.print(" Falha rc=");
        Serial.print(mqttClient.state());
        vTaskDelay(pdMS_TO_TICKS(5000)); 
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
  }
}

// ======= TAREFA EM LOOP: PUBLICAR DADOS MQTT ========
void TaskPublicarMQTT(void *pvParameters) {
    xSemaphoreTake(mqttConectadoSemaphore, portMAX_DELAY); 

    char epc_recebido[30];

    while (true) {
        if (mqttClient.connected()) {
            mqttClient.loop(); 
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue; 
        }

        if (xQueueReceive(rfidDataQueue, (void*)&epc_recebido, pdMS_TO_TICKS(100))) { 
            publicarMQTT(epc_recebido); 
        }
    }
}

// ======= SETUP ========
void setup() {
  Serial.begin(115200);

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  
  Serial.println("Iniciando: " __FILE__ " " __DATE__);

  NVS_SSID = "INFRA_CTISM";
  NVS_SENHA = "teste001";
  NVS_BROKER_MQTT = "mosquittoserver.controle.ufsm";

  preferences.begin("CERTS", true);
  CLIENT_CERT_PROGMEM = preferences.getString("tarnodePUB");
  CLIENT_KEY_PROGMEM = preferences.getString("tarnodePRIV");
  MQTT_CA_CERT_PROGMEM = preferences.getString("tarnodeCHAIN");
  preferences.end(); 

  MySerial.begin(115200, SERIAL_8N1, 16, 17);

  wifiConectadoSemaphore = xSemaphoreCreateBinary();
  mqttConectadoSemaphore = xSemaphoreCreateBinary();
  
  rfidDataQueue = xQueueCreate(50, sizeof(char[30])); 

  if (rfidReader.setTransmitPower(2500)) {
      Serial.println("Potência definida com SUCESSO.");
  }
  
  rfidReader.setDemodulatorParameters(0x03, 0x06, 0x01B0);
  rfidReader.setQueryParameters(1, 1, 1, 0);
  rfidReader.stopMultiplePolling(); 

  xTaskCreatePinnedToCore(TaskConectarWiFi, "Conectar-WiFi", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(TaskConectarMQTT, "Conectar-MQTT", 12288, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(TaskPublicarMQTT, "Publicar-MQTT", 8192, NULL, 5, NULL, 0);
}

// ======= LOOP PRINCIPAL DO ARDUINO (LEITURA) ========
void loop() {

  static unsigned long ultimoPoll = 0;
  static uint8_t responseBuffer[256];
  static const uint32_t GET_RESPONSE_TIMEOUT_MS = 1000;
  static unsigned long tempoDesconectado = 0;

  if (mqttClient.connected()) {
      tempoDesconectado = 0; 
      esp_task_wdt_reset(); 
  } else {

      if (tempoDesconectado == 0) tempoDesconectado = millis();
      
      if (millis() - tempoDesconectado > 120000) {
          Serial.println("MUITO TEMPO SEM MQTT. REINICIANDO...");
          ESP.restart();
      }
  }

  if (millis() - ultimoPoll >= T_INTERVALO_LEITURA_MS) {
      
      MySerial.flush(); 

      rfidReader.initiateSinglePolling(); 
      ultimoPoll = millis();
      
      if (rfidReader.getResponse(responseBuffer, 256, GET_RESPONSE_TIMEOUT_MS)) {
          
          if(rfidReader.hasValidTag(responseBuffer)) {
              uint8_t rssi;
              uint8_t epc[12];
              rfidReader.parseTagResponse(responseBuffer, rssi, epc);

              char epc_hex_str[25]; 
              
              snprintf(epc_hex_str, sizeof(epc_hex_str), 
                       "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                       epc[0], epc[1], epc[2], epc[3], epc[4], epc[5], 
                       epc[6], epc[7], epc[8], epc[9], epc[10], epc[11]
                      );

              if (strcmp(epc_hex_str, "000000000000000000000000") != 0) {
                  
                  Serial.printf("TAG: %s | RSSI: %d\n", epc_hex_str, rssi);
                  
                  if (xQueueSend(rfidDataQueue, (void*)&epc_hex_str, pdMS_TO_TICKS(10)) != pdPASS) { 
                      Serial.println("Fila Cheia!");
                  }
              }
          }
      }
  }
  
  vTaskDelay(pdMS_TO_TICKS(1)); 
}

// ======= FUNÇÃO PARA GRAVAR DADOS NO NVS (Executar apenas se necessário) ========
void setupNVS() {
    preferences.begin("mqtt_config", false); // false = modo leitura/escrita
    
    // Grava as chaves apenas se você chamar esta função no setup()
    preferences.putString("ssid", "INFRA_CTISM");          
    preferences.putString("senha", "teste001");      
    preferences.putString("broker_mqtt", "mosquittoserver.controle.ufsm"); 

    preferences.end(); 
    Serial.println("Configurações de rede e broker gravadas no NVS.");
}