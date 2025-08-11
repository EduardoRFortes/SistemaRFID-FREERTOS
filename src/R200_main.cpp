#include <WiFi.h>
#include <WiFiClientSecure.h> 
#include <PubSubClient.h>     
#include <ArduinoJson.h>
#include "R200.h"             
#include <time.h>             
#include <Preferences.h>      

// ====================================================================
// ======= HEADERS DO FREERTOS PARA ARDUINO IDE ========
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
// ====================================================================

// ====================================================================
// ======= DECLARAÇÕES DE OBJETOS E VARIÁVEIS GLOBAIS ========

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
R200 rfid;
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
const char* CLIENT_ID = "portal_entrada_5C01";
const char* TOPICO_MQTT = "/rfid/leituras";

const unsigned long T_JANELA_ATIVA_MODULO = 150;
const unsigned long T_INTERVALO_POLL_NA_JANELA = 50;
const unsigned long T_PAUSA_CURTA_POS_JANELA = 5;

const unsigned long T_SLOT_COMPLETO_MODULO = T_JANELA_ATIVA_MODULO + T_PAUSA_CURTA_POS_JANELA;
const unsigned long T_ESPERA_OUTROS_MODULOS = T_SLOT_COMPLETO_MODULO * 2;
const int MEU_SLOT_INDEX = 0;

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

  if (mqttClient.connected()) {
      JsonDocument doc;
      doc["epc"] = epc;
      time_t now = time(nullptr);
      doc["timestamp"] = now;
      doc["mqttId"] = CLIENT_ID;

      String jsonString;
      serializeJson(doc, jsonString);

      if (mqttClient.publish(TOPICO_MQTT, jsonString.c_str())) {
        Serial.println("Publicado via MQTT: " + jsonString);
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
  WiFi.begin(NVS_SSID.c_str(), NVS_SENHA.c_str()); 
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println("\nTask ConectarWiFi: Wi-Fi conectado! IP: " + WiFi.localIP().toString());
  
  Serial.println("Sincronizando hora com NTP....");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Falha ao obter o tempo via NTP. Certificados podem falhar!");
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

  espClient.setCACert(MQTT_CA_CERT_PROGMEM.c_str());
  espClient.setCertificate(CLIENT_CERT_PROGMEM.c_str());
  espClient.setPrivateKey(CLIENT_KEY_PROGMEM.c_str());

  // espClient.setVerifyHost(false); // Descomente para desativar a verificação de hostname (APENAS PARA TESTES!)

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
        Serial.println(". Tentando novamente em 5 segundos...");
        vTaskDelay(pdMS_TO_TICKS(5000));
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// ======= TAREFA DE LEITURA RFID ========
void TaskLeituraRFID(void *pvParameters) {
    Serial.println("Task LeituraRFID: Iniciando.");

    rfid.begin(&Serial2, 115200, 16, 17);
    rfid.setTransmissionPower(100);

    unsigned long initial_delay_ms = T_SLOT_COMPLETO_MODULO * MEU_SLOT_INDEX;
    if (initial_delay_ms > 0) {
        Serial.printf("Task LeituraRFID: Delay inicial de escalonamento de %lu ms.\n", initial_delay_ms);
        vTaskDelay(pdMS_TO_TICKS(initial_delay_ms));
    }
    Serial.println("Task LeituraRFID: Escalonamento inicial concluído.");

    unsigned long ultimoPollNaJanela = 0;
    char epc_buffer[30];

    while (1) {
        Serial.println("Task LeituraRFID: Iniciando Janela Ativa de Leitura...");
        unsigned long inicioJanelaAtiva = millis();
        ultimoPollNaJanela = inicioJanelaAtiva - T_INTERVALO_POLL_NA_JANELA; 

        while (millis() - inicioJanelaAtiva < T_JANELA_ATIVA_MODULO) {
            rfid.loop();

            if (millis() - ultimoPollNaJanela >= T_INTERVALO_POLL_NA_JANELA) {
                rfid.poll();
                ultimoPollNaJanela = millis();
            }

            rfid.loop();
            if (rfid.epc.length() > 0) {
                Serial.print("Task LeituraRFID: TAG LIDA: "); Serial.println(rfid.epc);
                rfid.epc.toCharArray(epc_buffer, sizeof(epc_buffer));
                
                if (xQueueSend(rfidDataQueue, (void*)&epc_buffer, pdMS_TO_TICKS(10)) != pdPASS) {
                    Serial.println("Task LeituraRFID: Falha ao enviar TAG para a fila MQTT (fila cheia?).");
                }
                rfid.epc = "";
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        Serial.println("Task LeituraRFID: Fim da Janela Ativa de Leitura.");

        vTaskDelay(pdMS_TO_TICKS(T_PAUSA_CURTA_POS_JANELA));

        Serial.printf("Task LeituraRFID: Iniciando Fase de Espera por %lu ms...\n", T_ESPERA_OUTROS_MODULOS);
        vTaskDelay(pdMS_TO_TICKS(T_ESPERA_OUTROS_MODULOS));
        Serial.println("Task LeituraRFID: Fim da Fase de Espera.");
    }
    vTaskDelete(NULL);
}

// ======= TAREFA EM LOOP: PUBLICAR DADOS MQTT (Recebe da fila) ========
void TaskPublicarMQTT(void *pvParameters) {
    Serial.println("Task PublicarMQTT: Aguardando conexão MQTT...");
    xSemaphoreTake(mqttConectadoSemaphore, portMAX_DELAY); 

    Serial.println("Task PublicarMQTT: MQTT conectado. Iniciando publicações.");

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

        if (xQueueReceive(rfidDataQueue, (void*)&epc_recebido, pdMS_TO_TICKS(100))) {
            Serial.print("Task PublicarMQTT: Recebido da fila para publicar: "); Serial.println(epc_recebido);
            publicarMQTT(String(epc_recebido)); 
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ======= SETUP ========
void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando: " __FILE__ " " __DATE__);

  NVS_SSID = "nersec";
  NVS_SENHA = "gremio123";
  NVS_BROKER_MQTT = "mosquittoserver.lan";

  preferences.begin("CERTS", true);

  CLIENT_CERT_PROGMEM = preferences.getString("tarnodePUB");
  CLIENT_KEY_PROGMEM = preferences.getString("tarnodePRIV");
  MQTT_CA_CERT_PROGMEM = preferences.getString("tarnodeCHAIN");
  
  preferences.end(); // Fecha o NVS



  // Verificações para garantir que os dados foram lidos do NVS
  if (NVS_SSID.length() == 0 || NVS_SENHA.length() == 0 || NVS_BROKER_MQTT.length() == 0) {
    Serial.println("ERRO: Credenciais de rede ou broker não encontradas no NVS! Execute setupNVS() uma vez.");
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); } 
  }

  Serial.println("Configurações de rede e broker lidas do NVS:");
  Serial.println("SSID: " + NVS_SSID);
  Serial.println("Broker: " + NVS_BROKER_MQTT);

  // Inicialização do módulo RFID
  rfid.begin(&Serial2, 115200, 16, 17); // Use as portas Tx/Rx corretas para seu RFID
  rfid.setTransmissionPower(100);

  // --- CRIAÇÃO DOS SEMÁFOROS ---
  wifiConectadoSemaphore = xSemaphoreCreateBinary();
  mqttConectadoSemaphore = xSemaphoreCreateBinary();
  
  if (wifiConectadoSemaphore == NULL || mqttConectadoSemaphore == NULL) {
    Serial.println("ERRO: Falha ao criar Semáforos! Sistema parado.");
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); } 
  }

  // --- CRIAÇÃO DA FILA DE DADOS RFID ---
  rfidDataQueue = xQueueCreate(5, sizeof(char[30])); 
  if (rfidDataQueue == NULL) {
      Serial.println("ERRO: Falha ao criar a fila de dados RFID! Sistema parado.");
      while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  }

  // --- CRIAÇÃO DAS TAREFAS FREE RTOS ---
  xTaskCreatePinnedToCore(
    TaskConectarWiFi, 
    "Conectar-WiFi",
    4096,
    NULL,
    5,
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

  xTaskCreatePinnedToCore(
    TaskLeituraRFID, 
    "Leitura-RFID",
    4096,
    NULL,
    3,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    TaskPublicarMQTT, 
    "Publicar-MQTT",
    8192,
    NULL,
    2,
    NULL,
    0
  );

  Serial.println("Todas as tarefas foram criadas pelo setup().");
  Serial.println("Sistema iniciado e escalonado (usando FreeRTOS tasks).");
}

// ======= LOOP PRINCIPAL DO ARDUINO ========
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1));
}

// ======= FUNÇÃO PARA GRAVAR DADOS NO NVS (NÃO SERÁ CHAMADA NESTA EXECUÇÃO) ========
// Mantenha esta função no código, mas ela não será executada, pois a chamada no setup()
// foi comentada para esta execução normal.
void setupNVS() {
    Serial.println("Gravando configurações no NVS...");
    preferences.begin("mqtt_config", false); 

    // --- CREDENCIAIS DE REDE ---
    preferences.putString("ssid", "nersec");          
    preferences.putString("senha", "gremio123");      
    preferences.putString("broker_mqtt", "mosquittoserver.lan"); 

    preferences.end(); 
    Serial.println("Configurações de rede e broker gravadas no NVS.");
}