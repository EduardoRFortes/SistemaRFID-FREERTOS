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

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// OBJETOS DA BIBLIOTECA ROBUSTA (RFIDR200)
HardwareSerial MySerial(2);
RFIDR200 rfidReader(MySerial, 115200);

Preferences preferences;

SemaphoreHandle_t wifiConectadoSemaphore;
SemaphoreHandle_t mqttConectadoSemaphore;
QueueHandle_t rfidDataQueue;
QueueHandle_t mqttCommandQueue;
// CONFIGURAÇÕES DE REDE E MQTT (LIDAS DO NVS)
String NVS_SSID;
String NVS_SENHA;
String NVS_BROKER_MQTT;

String CLIENT_CERT_PROGMEM;
String CLIENT_KEY_PROGMEM;
String MQTT_CA_CERT_PROGMEM;

// OUTRAS CONSTANTES GLOBAIS
const int PORTA_MQTT = 1883;
const char *CLIENT_ID = "portal_ESP_02";
const char *TOPICO_MQTT = "/rfid/leituras";

// DEFININDO O INTERVALO MÍNIMO ESTÁVEL DE LEITURA (200ms)
const unsigned long T_INTERVALO_LEITURA_MS = 200;

const char *ntpServer = "ntp1.ufsm.br";
// const char *ntpServer = "a.ntp.br";

// const char* ntpServer = "172.16.0.1";
const long gmtOffset_sec = -3 * 3600;
const int daylightOffset_sec = 0;

// Configuração do Watchdog (20 segundos)
#define WDT_TIMEOUT 20
#define led 2
typedef struct
{
  uint8_t payload[256];
  size_t length;
} MqttCommandMsg_t;

// ====================================================================

// ======= FUNÇÃO AUXILIAR DE PUBLICAÇÃO MQTT (Otimizada) ========
// Alterado para receber const char* para evitar criação de Strings desnecessárias na entrada
void publicarMQTT(const char *epcData)
{
  if (!mqttClient.connected())
  {
    Serial.println("PublicarMQTT: Conexão perdida. Tentando reconectar...");
    if (!mqttClient.connect(CLIENT_ID))
    {
      return;
    }
  }

  int epochValue = time(nullptr);
  Serial.println(epochValue);
  if (mqttClient.connected())
  {
    // Separa "EPC|RSSI" em campos individuais
    char epc[25] = {};
    int rssiVal = 0;
    const char *pipe = strchr(epcData, '|');
    if (pipe)
    {
      strncpy(epc, epcData, pipe - epcData);
      rssiVal = atoi(pipe + 1);
    }
    else
    {
      strncpy(epc, epcData, 24);
    }

    JsonDocument doc;
    doc["epc"] = epc;
    doc["mqttId"] = CLIENT_ID;
    doc["timestamp"] = epochValue;
    doc["rssi"] = rssiVal;

    String jsonString;
    serializeJson(doc, jsonString);

    if (mqttClient.publish(TOPICO_MQTT, jsonString.c_str()))
    {
      // Sucesso
    }
    else
    {
      Serial.println("Falha ao publicar via MQTT.");
    }
  }
}

void TaskReceberComando()
{
}
// ======= TAREFA DE INICIALIZAÇÃO E RECONEXÃO WI-FI ========
void TaskConectarWiFi(void *pvParameters)
{
  Serial.print("Task ConectarWiFi: Iniciando conexão Wi-Fi...");
  IPAddress primaryDNS(172, 16, 0, 1);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, primaryDNS, primaryDNS);
  WiFi.begin(NVS_SSID.c_str(), NVS_SENHA.c_str());

  Serial.println("Servidor DNS: ");
  Serial.println(WiFi.dnsIP());

  while (WiFi.status() != WL_CONNECTED)
  {
    vTaskDelay(pdMS_TO_TICKS(5000));
    Serial.print(".");
  }

  Serial.println("\nTask ConectarWiFi: Wi-Fi conectado! IP: " + WiFi.localIP().toString());

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;

  Serial.println("Solicitando hora ao servidor interno (172.16.0.1)...");

  if (!getLocalTime(&timeinfo, 5000))
  {
    Serial.println("ERRO: O servidor 172.16.0.1 não respondeu ao NTP.");
  }
  else
  {
    Serial.println(&timeinfo, "SUCESSO NTP INTERNO: %A, %B %d %Y %H:%M:%S");
  }
  // time_t now = time(nullptr);
  // struct tm utc;
  // gmtime_r(&now, &utc);
  // Serial.printf("epoch=%ld  UTC=%02d:%02d:%02d\n",
  //               (long)now, utc.tm_hour, utc.tm_min, utc.tm_sec);
  xSemaphoreGive(wifiConectadoSemaphore);
  Serial.println("Task ConectarWiFi: Concluída e se auto-deletando.");
  vTaskDelete(NULL);
}

// ======= TAREFA DE CONEXÃO E RECONEXÃO MQTT ========
void TaskConectarMQTT(void *pvParameters)
{
  Serial.println("Task ConectarMQTT: Aguardando Wi-Fi...");
  xSemaphoreTake(wifiConectadoSemaphore, portMAX_DELAY);

  const char *ca_cert = MQTT_CA_CERT_PROGMEM.c_str();
  const char *client_cert = CLIENT_CERT_PROGMEM.c_str();
  const char *client_key = CLIENT_KEY_PROGMEM.c_str();

  // Serial.printf("CLIENT CERT tamanho: %d\n", CLIENT_CERT_PROGMEM.length());
  // Serial.printf("CLIENT KEY tamanho: %d\n", CLIENT_KEY_PROGMEM.length());
  // Serial.printf("CA tamanho: %d\n", MQTT_CA_CERT_PROGMEM.length());

  // Serial.println("CLIENT CERT:");
  // Serial.println(client_cert );

  // Serial.println("CA CERT:");
  // Serial.println(ca_cert);

  // espClient.setCACert(ca_cert);
  // espClient.setInsecure();
  // espClient.setCertificate(client_cert);
  // espClient.setPrivateKey(client_key);

  // if (espClient.connect("172.22.48.145", 8883))
  // {
  //   Serial.println("TLS + certificado cliente OK");
  // }
  // else
  // {
  //   Serial.println("Falha TLS");
  // }
  mqttClient.setServer(NVS_BROKER_MQTT.c_str(), PORTA_MQTT);

  // char buf[256];
  // int err = espClient.lastError(buf, sizeof(buf));
  // Serial.printf("TLS error %d: %s\n", err, buf);

  while (true)
  {
    if (!mqttClient.connected())
    {
      Serial.print("Task ConectarMQTT: Conectando...");
      String currentClientId = String(CLIENT_ID) + "_" + String(esp_random(), HEX);

      if (mqttClient.connect(currentClientId.c_str()))
      {
        // Serial.println("\n[MQTT] Conectado!");
        mqttClient.subscribe("/rfid/comandos");
        xSemaphoreGive(mqttConectadoSemaphore);
      }
      else
      {
        Serial.print(" Falha rc=");
        Serial.print(mqttClient.state());
        vTaskDelay(pdMS_TO_TICKS(5000));
      }
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("\n>>> COMANDO RECEBIDO NO TOPICO ");
  Serial.print(topic);
  Serial.print(": ");
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.printf("%02X ", payload[i]); // Imprime cada byte em Hexadecimal
  }
  Serial.println();
  // Ignora se o comando for maior que nosso buffer
  if (length > sizeof(MqttCommandMsg_t::payload))
  {
    Serial.println("Comando MQTT muito grande!");
    return;
  }

  MqttCommandMsg_t newCmd;
  memcpy(newCmd.payload, payload, length);
  newCmd.length = length;

  // Envia o comando para a fila imediatamente (sem bloquear)
  if (xQueueSend(mqttCommandQueue, &newCmd, 0) != pdPASS)
  {
    Serial.println("Fila de comandos MQTT cheia!");
  }
}

void mqttCommandTask(void *pvParameters)
{
  MqttCommandMsg_t incomingCmd;
  const char *responseTopic = "rfid/respostas"; // Ajuste para o seu tópico

  while (1)
  {
    // Trava a Task aqui até que um comando chegue na fila (portMAX_DELAY)
    if (xQueueReceive(mqttCommandQueue, &incomingCmd, portMAX_DELAY) == pdTRUE)
    {
      // Quando recebe, chama a função que você já criou para falar com o R200
      // (Supondo que seu objeto R200 chame 'rfid' e o client 'mqttClient')
      Serial.printf("Tamanho recebido na Task: %d\n", incomingCmd.length);
      rfidReader.processMqttCommand(mqttClient,
                                    responseTopic,
                                    incomingCmd.payload,
                                    incomingCmd.length);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void configurarR200Padrao(RFIDR200 &rfidSerial)
{
  Serial.println("\n--- Iniciando Configuração do R200 via Hardcode ---");

  // 1. Comando Stop Polling (0x28)
  uint8_t cmdStop[] = {0xAA, 0x00, 0x28, 0x00, 0x00, 0x28, 0xDD};
  // 2. Comando Set Work Area (0x07) - Região US/BR (0x02)
  uint8_t cmdRegion[] = {0xAA, 0x00, 0x07, 0x00, 0x01, 0x02, 0x0A, 0xDD};
  // 3. Comando Set Transmit Power (0xB6) - 26 dBm (0x0A28)
  uint8_t cmdPower[] = {0xAA, 0x00, 0xB6, 0x00, 0x02, 0x0A, 0x28, 0xEA, 0xDD};

  // uint8_t cmdDemodulatorMax[] = {0xAA, 0x00, 0x11, 0x00, 0x04, 0x03, 0x06, 0x00, 0xA0, 0xBE, 0xDD};
  // uint8_t cmdDemodulatorMax[] = {0xAA, 0x00, 0x11, 0x00, 0x04, 0x03, 0x06, 0x00, 0xA0, 0xBE, 0xDD};

  // Função lambda corrigida usando as validações nativas da classe RFIDR200
  auto enviarEValidar = [&](uint8_t *cmd, size_t len, const char *nome)
  {
    rfidSerial.sendC(cmd, len); // Envia o comando para a UART

    uint8_t resp[30];

    // A getResponse já trava a task, aguarda a resposta e valida o Checksum/0xDD
    if (rfidSerial.getResponse(resp, sizeof(resp), 500))
    {
      // Calcula o tamanho real do frame recebido para imprimir corretamente
      size_t frameLength = 5 + (static_cast<size_t>(resp[3]) << 8 | static_cast<size_t>(resp[4])) + 2;

      Serial.printf("[%s] OK: ", nome);
      for (size_t j = 0; j < frameLength; j++)
      {
        Serial.printf("%02X ", resp[j]);
      }
      Serial.println();
      return true;
    }

    Serial.printf("[%s] FALHA OU TIMEOUT\n", nome);
    return false;
  };

  enviarEValidar(cmdStop, sizeof(cmdStop), "Stop Polling");
  vTaskDelay(pdMS_TO_TICKS(100));

  enviarEValidar(cmdRegion, sizeof(cmdRegion), "Set Region (US/BR)");
  vTaskDelay(pdMS_TO_TICKS(100));

  // enviarEValidar(cmdDemodulatorMax, sizeof(cmdDemodulatorMax), "Set Threshold: ");
  vTaskDelay(pdMS_TO_TICKS(100));

  // enviarEValidar(cmdPower, sizeof(cmdPower), "Set Power (26 dBm)");
  Serial.println("--- Configuração do R200 Concluída ---\n");
}

// ======= TAREFA EM LOOP: PUBLICAR DADOS MQTT ========
void TaskPublicarMQTT(void *pvParameters)
{
  xSemaphoreTake(mqttConectadoSemaphore, portMAX_DELAY);

  char epc_recebido[30];

  while (true)
  {
    if (mqttClient.connected())
    {
      mqttClient.loop();
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (xQueueReceive(rfidDataQueue, (void *)&epc_recebido, pdMS_TO_TICKS(100)))
    {
      publicarMQTT(epc_recebido);
    }
  }
}

void config()
{
}

// ======= SETUP ========
void setup()
{
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  Serial.println("Iniciando: " __FILE__ " " __DATE__);

  NVS_SSID = "nersec";
  NVS_SENHA = "N3R5eC5c2025";

  // NVS_SSID = "INFRA_CTISM";
  // NVS_SENHA = "teste001";

  NVS_BROKER_MQTT = "172.22.48.145";
  //: NVS_BROKER_MQTT = "mosquittoserver.controle.ufsm";

  preferences.begin("CERTS", true);
  CLIENT_CERT_PROGMEM = preferences.getString("tarnodePUB");
  CLIENT_KEY_PROGMEM = preferences.getString("tarnodePRIV");
  MQTT_CA_CERT_PROGMEM = preferences.getString("tarnodeCHAIN");
  preferences.end();

  // Serial.printf("client.crt: %d bytes\n", CLIENT_CERT_PROGMEM.length());
  // Serial.printf("client.crt: %d bytes\n", CLIENT_CERT_PROGMEM.length());
  // Serial.printf("client.key: %d bytes\n", CLIENT_KEY_PROGMEM.length());
  // Serial.printf("ca.crt: %d bytes\n", MQTT_CA_CERT_PROGMEM.length());

  // Serial.println(CLIENT_CERT_PROGMEM.startsWith("-----BEGIN CERTIFICATE-----")
  //                    ? "CLIENT CERT OK"
  //                    : "CLIENT CERT INVALID");

  // Serial.println(CLIENT_KEY_PROGMEM.startsWith("-----BEGIN")
  //                    ? "CLIENT KEY OK"
  //                    : "CLIENT KEY INVALID");

  // Serial.println(MQTT_CA_CERT_PROGMEM.startsWith("-----BEGIN CERTIFICATE-----")
  //                    ? "CA CERT OK"
  //                    : "CA CERT INVALID");
  MySerial.begin(115200, SERIAL_8N1, 16, 17);

  wifiConectadoSemaphore = xSemaphoreCreateBinary();
  mqttConectadoSemaphore = xSemaphoreCreateBinary();

  rfidDataQueue = xQueueCreate(50, sizeof(char[30]));
  // mqttCommandQueue = xQueueCreate(10, sizeof(MqttCommandMsg_t)); // <-- INICIALIZA NOVA FILA

  if (rfidReader.setTransmitPower(2600))
  {
    Serial.println("Potência definida com SUCESSO.");
  }

  //   0x0080 (mais sensível, muito ruído)
  // 0x00A0
  // 0x00F0
  // 0x0120
  // 0x0150
  // 0x0180
  // 0x01B0 (padrão do manual)
  // 0x01E0
  // 0x0210 (menos sensível, mais estável)

  // rfidReader.setDemodulatorParameters(0x03, 0x06, 0x01B0);
  // rfidReader.setDemodulatorParameters(0x03, 0x06, 0x01B0);
  rfidReader.setDemodulatorParameters(0x03, 0x06, 0x0150);
  rfidReader.setQueryParameters(1, 1, 1, 0);
  rfidReader.stopMultiplePolling();

  rfidReader.getWorkArea();
  configurarR200Padrao(rfidReader);
  // mqttClient.setCallback(mqttCallback);

  // xTaskCreatePinnedToCore(mqttCommandTask, "mqtt_cmd_task", 4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(TaskConectarWiFi, "Conectar-WiFi", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(TaskConectarMQTT, "Conectar-MQTT", 12288, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(TaskPublicarMQTT, "Publicar-MQTT", 8192, NULL, 4, NULL, 0);
}

// ======= LOOP PRINCIPAL DO ARDUINO (LEITURA) ========
void loop()
{

  static unsigned long ultimoPoll = 0;
  static uint8_t responseBuffer[256];
  static const uint32_t GET_RESPONSE_TIMEOUT_MS = 1000;
  static unsigned long tempoDesconectado = 0;

  if (mqttClient.connected())
  {
    tempoDesconectado = 0;
    esp_task_wdt_reset();
  }
  else
  {

    if (tempoDesconectado == 0)
      tempoDesconectado = millis();

    if (millis() - tempoDesconectado > 120000)
    {
      Serial.println("MUITO TEMPO SEM MQTT. REINICIANDO...");
      ESP.restart();
    }
  }

  // if (false)
  if (millis() - ultimoPoll >= T_INTERVALO_LEITURA_MS)
  {

    MySerial.flush();

    rfidReader.initiateSinglePolling();
    ultimoPoll = millis();

    if (rfidReader.getResponse(responseBuffer, 256, GET_RESPONSE_TIMEOUT_MS))
    {

      if (rfidReader.hasValidTag(responseBuffer))
      {
        uint8_t rssi;
        uint8_t epc[12];
        rfidReader.parseTagResponse(responseBuffer, rssi, epc);

        char epc_hex_str[30];

        snprintf(epc_hex_str, 25,
                 "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                 epc[0], epc[1], epc[2], epc[3], epc[4], epc[5],
                 epc[6], epc[7], epc[8], epc[9], epc[10], epc[11]);

        if (strcmp(epc_hex_str, "000000000000000000000000") != 0)
        {

          Serial.printf("TAG: %s | RSSI: %d\n", epc_hex_str, rssi);
          digitalWrite(led, !digitalRead(led));
          // Deduplicação: descarta mesma tag vista há menos de 5s
          static char dedupEpc[8][25] = {};
          static unsigned long dedupMs[8] = {0};

          bool shouldPublish = true;
          int matchSlot = -1;
          int freeSlot = -1;
          int oldestSlot = 0;

          for (int i = 0; i < 8; i++)
          {
            if (dedupEpc[i][0] != '\0' && strcmp(dedupEpc[i], epc_hex_str) == 0)
            {
              matchSlot = i;
              if (millis() - dedupMs[i] < 5000UL)
              {
                shouldPublish = false;
              }
              else
              {
                dedupMs[i] = millis();
              }
              break;
            }
            if (dedupEpc[i][0] == '\0' && freeSlot < 0)
              freeSlot = i;
            if (dedupMs[i] < dedupMs[oldestSlot])
              oldestSlot = i;
          }

          if (shouldPublish)
          {
            if (matchSlot < 0)
            {
              int slot = (freeSlot >= 0) ? freeSlot : oldestSlot;
              strncpy(dedupEpc[slot], epc_hex_str, 24);
              dedupEpc[slot][24] = '\0';
              dedupMs[slot] = millis();
            }

            // Anexa RSSI ao final: "XXXXXXXXXXXXXXXXXXXXXXXX|rrr"
            snprintf(epc_hex_str + 24, 6, "|%d", rssi);

            if (xQueueSend(rfidDataQueue, (void *)&epc_hex_str, pdMS_TO_TICKS(10)) != pdPASS)
            {
              Serial.println("Fila Cheia!");
            }
          }
        }
      }
    }
  }

  vTaskDelay(pdMS_TO_TICKS(10));
}

// ======= FUNÇÃO PARA GRAVAR DADOS NO NVS (Executar apenas se necessário) ========
void setupNVS()
{
  preferences.begin("mqtt_config", false); // false = modo leitura/escrita

  // Grava as chaves apenas se você chamar esta função no setup()
  preferences.putString("ssid", "INFRA_CTISM");
  preferences.putString("senha", "teste001");
  preferences.putString("broker_mqtt", "mosquittoserver.controle.ufsm");

  preferences.end();
  Serial.println("Configurações de rede e broker gravadas no NVS.");
}