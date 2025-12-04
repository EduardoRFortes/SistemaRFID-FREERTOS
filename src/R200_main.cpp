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

// ====================================================================
// ======= PROTÓTIPOS DE FUNÇÃO (CORREÇÃO DO ERRO DE ESCOPO) ===========
// O compilador precisa saber que essas funções existem antes de serem chamadas no setup()
void TaskConectarWiFi(void *pvParameters);
void TaskConectarMQTT(void *pvParameters);
void TaskPublicarMQTT(void *pvParameters);
// A lógica de leitura está no loop() principal agora
// ====================================================================

// ====================================================================
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

// CONSTANTES DO WATCH DOG


// DEFININDO O INTERVALO MÍNIMO ESTÁVEL DE LEITURA (200ms)
const unsigned long T_INTERVALO_LEITURA_MS = 200; 

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -3 * 3600;
const int daylightOffset_sec = 0;
// ====================================================================

// ======= FUNÇÃO AUXILIAR DE PUBLICAÇÃO MQTT ========
void publicarMQTT(const String& epc) {
  if (!mqttClient.connected()) {
    Serial.println("PublicarMQTT (auxiliar): Conexão MQTT perdida. Tentando reconectar para esta pub...");
    mqttClient.connect(CLIENT_ID); 
  }

  int epochValue = time(nullptr);

  if (mqttClient.connected()) {
      JsonDocument doc;
      doc["epc"] = epc;
      doc["mqttId"] = CLIENT_ID; 
      doc["timestamp"] = epochValue;

      String jsonString;
      serializeJson(doc, jsonString);

      if (mqttClient.publish(TOPICO_MQTT, jsonString.c_str())) {
        //Serial.println("Publicado via MQTT: " + jsonString);
      } else {
        Serial.println("Falha ao publicar via MQTT (cliente conectado).");
      }
  } else {
      Serial.println("PublicarMQTT (auxiliar): Não foi possível publicar: Cliente não conectado.");
  }
}

// ======= TAREFA DE INICIALIZAÇÃO E RECONEXÃO WI-FI ========
void TaskConectarWiFi(void *pvParameters) {
  Serial.print("Task ConectarWiFi: Iniciando conexão Wi-Fi...");
  IPAddress primaryDNS(172, 16, 0, 1);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, primaryDNS, primaryDNS);
  WiFi.begin(NVS_SSID.c_str(), NVS_SENHA.c_str()); 
  // 172.16.5.11
  // 88:13:BF:C8:6A:68

  Serial.println("Servidor DNS: ");
  Serial.println(WiFi.dnsIP());
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println("\nTask ConectarWiFi: Wi-Fi conectado! IP: " + WiFi.localIP().toString());

  Serial.println("\nEndereço MAC:");
  Serial.println(WiFi.macAddress());
  
  // Reabilitando o NTP para correta validação de certificado
  Serial.println("Sincronizando hora com NTP....");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Falha ao obter o tempo via NTP.");
  } else {
    Serial.println(&timeinfo, "Hora atual: %A, %B %d %Y %H:%M:%S");
  }


  xSemaphoreGive(wifiConectadoSemaphore); 
  Serial.println("Task ConectarWiFi: Concluída e se auto-deletando.");

  vTaskDelete(NULL);
}

// ======= TAREFA DE CONEXÃO E RECONEXÃO MQTT ========
void TaskConectarMQTT(void *pvParameters) {
  Serial.println("Task ConectarMQTT: Aguardando conexão Wi-Fi...");
  xSemaphoreTake(wifiConectadoSemaphore, portMAX_DELAY); 
  Serial.println("Task ConectarMQTT: Wi-Fi conectado, iniciando loop de conexão MQTT...");

  const char* ca_cert = MQTT_CA_CERT_PROGMEM.c_str();
  const char* client_cert = CLIENT_CERT_PROGMEM.c_str();
  const char* client_key = CLIENT_KEY_PROGMEM.c_str();

  // Autenticação Mútua (mTLS)
  espClient.setCACert(ca_cert);
  espClient.setCertificate(client_cert);
  espClient.setPrivateKey(client_key);
  
  mqttClient.setServer(NVS_BROKER_MQTT.c_str(), PORTA_MQTT); 

  while (true) {
    if (!mqttClient.connected()) {
      Serial.print("Task ConectarMQTT: Tentando conectar ao broker MQTT...");
      String currentClientId = String(CLIENT_ID) + "_" + String(esp_random(), HEX); 
      
      if (mqttClient.connect(currentClientId.c_str())) {
        Serial.println("\n[Task ConectarMQTT] Conectado ao MQTT!");
        xSemaphoreGive(mqttConectadoSemaphore); 
      } else {
        Serial.print(" [Task ConectarMQTT] Falha na conexão, rc=");
        Serial.print(mqttClient.state());
        vTaskDelay(pdMS_TO_TICKS(10000));
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(100)); 
    }
  }
}

// ======= TAREFA EM LOOP: PUBLICAR DADOS MQTT (Recebe da fila) ========
void TaskPublicarMQTT(void *pvParameters) {
    xSemaphoreTake(mqttConectadoSemaphore, portMAX_DELAY); 

    char epc_recebido[30];

    while (true) {
        if (mqttClient.connected()) {
            mqttClient.loop(); 
        } else {
            Serial.println("Task PublicarMQTT: MQTT desconectado. Aguardando reconexão...");
            xSemaphoreTake(mqttConectadoSemaphore, portMAX_DELAY);
            Serial.println("Task PublicarMQTT: MQTT Reconectado (sinalizado).");
            continue; 
        }

        // Aumentando o timeout de recebimento para 500ms
        if (xQueueReceive(rfidDataQueue, (void*)&epc_recebido, pdMS_TO_TICKS(500))) { 
            publicarMQTT(String(epc_recebido)); 
        }
    }
}

// ======= SETUP ========
void setup() {
  Serial.begin(115200);

  esp_task_wdt_init(20, true);

  
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

  // --- CRIAÇÃO DOS SEMÁFOROS E FILAS ---
  wifiConectadoSemaphore = xSemaphoreCreateBinary();
  mqttConectadoSemaphore = xSemaphoreCreateBinary();
  
  if (wifiConectadoSemaphore == NULL || mqttConectadoSemaphore == NULL) {
    Serial.println("ERRO: Falha ao criar Semáforos! Sistema parado.");
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); } 
  }

  // Tamanho da fila aumentado para 20
  rfidDataQueue = xQueueCreate(20, sizeof(char[30])); 
  if (rfidDataQueue == NULL) {
      Serial.println("ERRO: Falha ao criar a fila de dados RFID! Sistema parado.");
      while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  }

  // --- CONFIGURAÇÃO INICIAL DO RFID (FORA DA THREAD) ---
  // A configuração é executada no setup() para ser feita apenas uma vez.
  
  // 1. POTÊNCIA (2500 para estabilidade de energia)
  if (rfidReader.setTransmitPower(2500)) {
      Serial.println("Potência definida com SUCESSO.");
  } else {
      Serial.println("FALHA ao definir a potência! (Ignorando falha para continuar)");
  }
  
  // 2. MODO DE ALTA SENSIBILIDADE
  uint8_t mixerGain = 0x03; 
  uint8_t ifGain = 0x06;    
  uint16_t threshold = 0x01B0;
  if (rfidReader.setDemodulatorParameters(mixerGain, ifGain, threshold)) {
      Serial.println("Parâmetros do demodulador definidos com SUCESSO.");
  } else {
      Serial.println("FALHA ao definir os parâmetros do demodulador! (Ignorando falha)");
  }
  
  // 3. ALGORITMO OTIMIZADO
  if(rfidReader.setQueryParameters(1, 1, 1, 0)) {
      Serial.println("Parâmetros de Query definidos com SUCESSO.");
  } else {
      Serial.println("FALHA ao definir os parâmetros de Query! (Ignorando falha)");
  }
  
  // 4. Garante o modo Single Poll
  rfidReader.stopMultiplePolling(); 


  // --- CRIAÇÃO DAS TAREFAS FREE RTOS ---
  xTaskCreatePinnedToCore(
    TaskConectarWiFi, 
    "Conectar-WiFi",
    4096,
    NULL,
    5, // Prioridade 5
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    TaskConectarMQTT, 
    "Conectar-MQTT",
    12288,
    NULL,
    4,
    NULL,
    1
  );

  // Não precisamos mais da TaskLeituraRFID; a lógica está no loop()
  
  xTaskCreatePinnedToCore(
    TaskPublicarMQTT, 
    "Publicar-MQTT",
    8192,
    NULL,
    5, // Prioridade 5 (Máxima)
    NULL,
    0
  );

  Serial.println("Todas as tarefas foram criadas pelo setup().");
  Serial.println("Sistema iniciado e escalonado (usando FreeRTOS tasks).");
}

// ======= LOOP PRINCIPAL DO ARDUINO (LEITURA) ========
void loop() {
  // Lógica de Leitura RFID no Loop Principal
  static unsigned long ultimoPoll = 0;
  static uint8_t responseBuffer[256];
  static const uint32_t GET_RESPONSE_TIMEOUT_MS = 1000; 

  if (millis() - ultimoPoll >= T_INTERVALO_LEITURA_MS) {
      
      MySerial.flush(); 

      // 2. Envia o comando de POLL MANUAL
      rfidReader.initiateSinglePolling(); 
      ultimoPoll = millis();
      
      if (rfidReader.getResponse(responseBuffer, 256, GET_RESPONSE_TIMEOUT_MS)) {
        esp_task_wdt_add(NULL);
          if(rfidReader.hasValidTag(responseBuffer)) {
              uint8_t rssi;
              uint8_t epc[12];
              rfidReader.parseTagResponse(responseBuffer, rssi, epc);

              String epc_string = "";
              for (int i = 0; i < 12; ++i) {
                  if (epc[i] < 0x10) epc_string += "0";
                  epc_string += String(epc[i], HEX);
              }
              
              epc_string.toUpperCase(); 

              if (epc_string.startsWith("0000000000000000")) {
              } else {
                  Serial.printf("TAG LIDA: %s | RSSI: %d\n", epc_string.c_str(), rssi);
                  
                  char epc_buffer[30];
                  epc_string.substring(0, 24).toCharArray(epc_buffer, sizeof(epc_buffer));
                  
                  // 4. Envia para a fila MQTT
                  if (xQueueSend(rfidDataQueue, (void*)&epc_buffer, pdMS_TO_TICKS(10)) != pdPASS) { 
                      Serial.println("Falha ao enviar TAG para a fila. (Fila Cheia)");
                  }
              }
          }
        esp_task_wdt_reset();
      }
  }
  
  // Damos tempo para o FreeRTOS rodar as outras tarefas
  vTaskDelay(pdMS_TO_TICKS(1)); 
}

// ======= FUNÇÃO PARA GRAVAR DADOS NO NVS (NÃO SERÁ CHAMADA NESTA EXECUÇÃO) ========
void setupNVS() {
    preferences.begin("mqtt_config", false); 
    
    preferences.putString("ssid", "INFRA_CTISM");          
    preferences.putString("senha", "teste001");      
    preferences.putString("broker_mqtt", "mosquittoserver.controle.ufsm"); 

    preferences.end(); 
    Serial.println("Configurações de rede e broker gravadas no NVS.");
    
}