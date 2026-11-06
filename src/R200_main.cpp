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
void publicarMQTT(const char* epcData) {
  if (!mqttClient.connected()) {
    Serial.println("PublicarMQTT: Conexão perdida. Tentando reconectar...");
    if(!mqttClient.connect(CLIENT_ID)) {
       return;
    }
  }

  int epochValue = time(nullptr);
  Serial.println(epochValue);
  if (mqttClient.connected()) {
      // Separa "EPC|RSSI" em campos individuais
      char epc[25] = {};
      int rssiVal  = 0;
      const char* pipe = strchr(epcData, '|');
      if (pipe) {
          strncpy(epc, epcData, pipe - epcData);
          rssiVal = atoi(pipe + 1);
      } else {
          strncpy(epc, epcData, 24);
      }

      JsonDocument doc;
      doc["epc"]       = epc;
      doc["mqttId"]    = CLIENT_ID;
      doc["timestamp"] = epochValue;
      doc["rssi"]      = rssiVal;

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


// ======= MODO BANCADA (depuração do R200 pela serial USB) ========
// Comandos (terminar com Enter):
//   g          lê a potência atual do módulo (0xB7)
//   p<valor>   define a potência em centi-dBm e lê de volta (ex.: p2000 = 20 dBm)
//   m<seg>     mede por <seg> segundos: tentativas, leituras, taxa e RSSI por EPC
//   d<n>       n pollings únicos com o quadro cru da resposta
//   l          liga/desliga a leitura contínua normal do loop
// Sem NVS/certificados o MQTT nunca conecta e o loop reiniciaria a cada 2 min;
// com BANCADA=1 esse reinício fica desligado. REMOVER na implantação.
#define BANCADA 1

static bool leituraContinua = true;

static uint8_t checksumBancada(const uint8_t* c, size_t n) {
    uint8_t s = 0;
    for (size_t i = 1; i < n; i++) s += c[i];
    return s;
}

static void imprimeHex(const char* rotulo, const uint8_t* b, size_t n) {
    Serial.printf("%s", rotulo);
    for (size_t i = 0; i < n; i++) Serial.printf("%02X ", b[i]);
    Serial.println();
}

// Envia um comando e imprime o quadro cru da resposta. Devolve true se chegou quadro.
static bool comandoBancada(uint8_t cmd, const uint8_t* payload, uint16_t plen, uint8_t* resp, size_t rlen) {
    uint8_t f[32];
    f[0] = 0xAA; f[1] = 0x00; f[2] = cmd; f[3] = plen >> 8; f[4] = plen & 0xFF;
    for (uint16_t i = 0; i < plen; i++) f[5 + i] = payload[i];
    f[5 + plen] = checksumBancada(f, 5 + plen);
    f[6 + plen] = 0xDD;
    while (MySerial.available()) MySerial.read();
    MySerial.write(f, 7 + plen);
    imprimeHex("  >> ", f, 7 + plen);
    memset(resp, 0, rlen);
    bool ok = rfidReader.getResponse(resp, rlen, 500);
    if (!ok) { Serial.println("  << TIMEOUT (nenhum quadro do R200)"); return false; }
    size_t n = 7 + ((resp[3] << 8) | resp[4]);
    if (n > rlen) n = rlen;
    imprimeHex("  << ", resp, n);
    return true;
}

static void lerPotencia() {
    uint8_t r[32];
    if (comandoBancada(0xB7, nullptr, 0, r, sizeof(r)) && r[2] == 0xB7) {
        uint16_t p = (r[5] << 8) | r[6];
        Serial.printf("POTENCIA lida: %u centi-dBm (%.2f dBm)\n", p, p / 100.0);
    }
}

static void definirPotencia(uint16_t p) {
    uint8_t pl[2] = { (uint8_t)(p >> 8), (uint8_t)(p & 0xFF) };
    uint8_t r[32];
    Serial.printf("Definindo potencia %u centi-dBm\n", p);
    if (comandoBancada(0xB6, pl, 2, r, sizeof(r))) {
        Serial.printf("  status=0x%02X (0x00 = aceito)\n", r[5]);
    }
    lerPotencia();
}

static void medir(uint32_t seg) {
    struct Est { char epc[25]; uint32_t n; uint32_t soma; uint8_t mn, mx; };
    Est est[8]; int nEst = 0;
    uint32_t tent = 0, hits = 0;
    uint8_t r[64];
    uint32_t fim = millis() + seg * 1000UL;
    Serial.printf("Medindo por %us...\n", (unsigned)seg);
    while ((int32_t)(millis() - fim) < 0) {
        esp_task_wdt_reset();
        while (MySerial.available()) MySerial.read();
        rfidReader.initiateSinglePolling();
        tent++;
        if (!rfidReader.getResponse(r, sizeof(r), 150)) continue;
        if (r[2] != 0x22) continue;           // 0xFF = sem etiqueta
        char e[25];
        snprintf(e, sizeof(e), "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                 r[8], r[9], r[10], r[11], r[12], r[13], r[14], r[15], r[16], r[17], r[18], r[19]);
        hits++;
        int k = -1;
        for (int i = 0; i < nEst; i++) if (!strcmp(est[i].epc, e)) { k = i; break; }
        if (k < 0 && nEst < 8) { k = nEst++; strcpy(est[k].epc, e); est[k].n = 0; est[k].soma = 0; est[k].mn = 255; est[k].mx = 0; }
        if (k >= 0) { est[k].n++; est[k].soma += r[5]; if (r[5] < est[k].mn) est[k].mn = r[5]; if (r[5] > est[k].mx) est[k].mx = r[5]; }
    }
    Serial.printf("RESULTADO: tentativas=%u leituras=%u taxa=%.1f%% (%.1f leituras/s)\n",
                  tent, hits, tent ? 100.0 * hits / tent : 0.0, hits / (double)seg);
    for (int i = 0; i < nEst; i++)
        Serial.printf("  EPC %s  n=%u  RSSI medio=%.1f min=%u max=%u\n",
                      est[i].epc, est[i].n, est[i].soma / (double)est[i].n, est[i].mn, est[i].mx);
    if (!nEst) Serial.println("  nenhuma etiqueta lida");
}

// Um polling único com o quadro cru da resposta (diagnóstico)
static void pollingCru(int vezes) {
    uint8_t r[64];
    for (int i = 0; i < vezes; i++) {
        esp_task_wdt_reset();
        while (MySerial.available()) MySerial.read();
        uint32_t t0 = millis();
        rfidReader.initiateSinglePolling();
        memset(r, 0, sizeof(r));
        bool ok = rfidReader.getResponse(r, sizeof(r), 1000);
        uint32_t dt = millis() - t0;
        if (!ok) { Serial.printf("  poll %d: TIMEOUT (%ums)\n", i, (unsigned)dt); continue; }
        size_t n = 7 + ((r[3] << 8) | r[4]); if (n > sizeof(r)) n = sizeof(r);
        Serial.printf("  poll %d (%ums): ", i, (unsigned)dt);
        imprimeHex("", r, n);
    }
}

static void tratarSerialBancada() {
    static String linha;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c != '\n') { linha += c; continue; }
        linha.trim();
        if (linha.length()) {
            char k = linha[0];
            if (k == 'g') lerPotencia();
            else if (k == 'p') definirPotencia((uint16_t)linha.substring(1).toInt());
            else if (k == 'm') medir(max(1L, linha.substring(1).toInt()));
            else if (k == 'd') pollingCru(max(1L, linha.substring(1).toInt()));
            else if (k == 'l') { leituraContinua = !leituraContinua; Serial.printf("leitura continua: %s\n", leituraContinua ? "ligada" : "desligada"); }
            else Serial.println("comandos: g | p<centi-dBm> | m<seg> | d<n> | l");
        }
        linha = "";
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

  // Bancada: mostra o quadro cru em vez de falhar em silêncio
  definirPotencia(1000);  // bancada: sobe baixo para não derrubar o USB; subir com o comando p
  
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
      
      if (!BANCADA && millis() - tempoDesconectado > 120000) {
          Serial.println("MUITO TEMPO SEM MQTT. REINICIANDO...");
          ESP.restart();
      }
  }

  tratarSerialBancada();

  if (leituraContinua && millis() - ultimoPoll >= T_INTERVALO_LEITURA_MS) {
      
      MySerial.flush(); 

      rfidReader.initiateSinglePolling(); 
      ultimoPoll = millis();
      
      if (rfidReader.getResponse(responseBuffer, 256, GET_RESPONSE_TIMEOUT_MS)) {
          
          if(rfidReader.hasValidTag(responseBuffer)) {
              uint8_t rssi;
              uint8_t epc[12];
              rfidReader.parseTagResponse(responseBuffer, rssi, epc);

              char epc_hex_str[30];

              snprintf(epc_hex_str, 25,
                       "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                       epc[0], epc[1], epc[2], epc[3], epc[4], epc[5],
                       epc[6], epc[7], epc[8], epc[9], epc[10], epc[11]
                      );

              if (strcmp(epc_hex_str, "000000000000000000000000") != 0) {

                  Serial.printf("TAG: %s | RSSI: %d\n", epc_hex_str, rssi);

                  // Deduplicação: descarta mesma tag vista há menos de 5s
                  static char          dedupEpc[8][25] = {};
                  static unsigned long dedupMs[8]      = {0};

                  bool shouldPublish = true;
                  int  matchSlot  = -1;
                  int  freeSlot   = -1;
                  int  oldestSlot = 0;

                  for (int i = 0; i < 8; i++) {
                      if (dedupEpc[i][0] != '\0' && strcmp(dedupEpc[i], epc_hex_str) == 0) {
                          matchSlot = i;
                          if (millis() - dedupMs[i] < 5000UL) {
                              shouldPublish = false;
                          } else {
                              dedupMs[i] = millis();
                          }
                          break;
                      }
                      if (dedupEpc[i][0] == '\0' && freeSlot < 0) freeSlot = i;
                      if (dedupMs[i] < dedupMs[oldestSlot]) oldestSlot = i;
                  }

                  if (shouldPublish) {
                      if (matchSlot < 0) {
                          int slot = (freeSlot >= 0) ? freeSlot : oldestSlot;
                          strncpy(dedupEpc[slot], epc_hex_str, 24);
                          dedupEpc[slot][24] = '\0';
                          dedupMs[slot] = millis();
                      }

                      // Anexa RSSI ao final: "XXXXXXXXXXXXXXXXXXXXXXXX|rrr"
                      snprintf(epc_hex_str + 24, 6, "|%d", rssi);

                      if (xQueueSend(rfidDataQueue, (void*)&epc_hex_str, pdMS_TO_TICKS(10)) != pdPASS) {
                          Serial.println("Fila Cheia!");
                      }
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