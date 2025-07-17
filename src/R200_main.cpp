#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "R200.h"
#include <WiFiClientSecure.h>
#include <time.h>

// ======= DEFINIÇÕES PARA TIME.H ========

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -3 * 3600;
const int daylightOffset_sec = 0;

// ======= HEADERS DO FREERTOS PARA ARDUINO IDE ========
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ======= CONFIGURAÇÕES DE REDE E MQTT ========
const char* SSID = "nersec";
const char* SENHA = "gremio123";
const char* BROKER_MQTT = "mosquittoserver.lan"; // Seu broker MQTT
const int PORTA_MQTT = 8883;

// ===================================================================================
// ======= IMPORTANTE: CLIENT_ID DEVE SER ÚNICO PARA CADA ESP32/MÓDULO! =======
const char* CLIENT_ID = "ESP32_RFID_Client_1"; // <-- MUDE PARA CADA MÓDULO!
// ===================================================================================

const char* TOPICO_MQTT = "/rfid/leituras"; // Tópico para publicar leituras RFID

// ======= CONFIGURAÇÕES DE TEMPO PARA O SEQUENCIAMENTO (PARA LEITURA RFID) ========
const unsigned long T_JANELA_ATIVA_MODULO = 150;       // Duração da "janela de oportunidade" de leitura deste módulo (ms)
const unsigned long T_INTERVALO_POLL_NA_JANELA = 50; // A cada quantos ms chamar rfid.poll() DENTRO da janela ativa
const unsigned long T_PAUSA_CURTA_POS_JANELA = 5;    // Pequena pausa após a janela ativa deste módulo (ms)

const unsigned long T_SLOT_COMPLETO_MODULO = T_JANELA_ATIVA_MODULO + T_PAUSA_CURTA_POS_JANELA;
const unsigned long T_ESPERA_OUTROS_MODULOS = T_SLOT_COMPLETO_MODULO * 2; // Ex: 155 * 2 = 310 ms para 2 outros módulos

// ===================================================================================
// ======= AJUSTE ESTE VALOR PARA CADA MÓDULO (0, 1, 2, ...) ========
const int MEU_SLOT_INDEX = 0; // MÓDULO 1: 0; MÓDULO 2: 1; MÓDULO 3: 2
// ===================================================================================

// ======= OBJETOS GLOBAIS ========
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
R200 rfid;

// ======= CERTIFICADOS DIGITAIS ========

#define client_cert "-----BEGIN CERTIFICATE-----\n"\
"MIICSTCCAe+gAwIBAgIRAOjlIq1MGLdag3pedDIBlh8wCgYIKoZIzj0EAwIwNDEL\n"\
"MAkGA1UEBhMCQlIxFDASBgNVBAoMC05FUlNFQy1VRlNNMQ8wDQYDVQQDDAZjYS1s\n"\
"YW4wHhcNMjUwNzE2MTcyNzMwWhcNMjgwNzE1MTcyNzMwWjBHMQswCQYDVQQGEwJC\n"\
"UjELMAkGA1UECAwCUlMxFDASBgNVBAoMC05FUlNFQy1VRlNNMRUwEwYDVQQDDAx0\n"\
"YXJub2RlMS5sYW4wWTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAAQ9/PpLDbxYasgq\n"\
"NfWsTnmoopCKqT34x8cD2vxyjBqf0K95vA6VGgh+T0Uj7RJE1PWYwjYxTEucMGYi\n"\
"R3YRJBZco4HOMIHLMAwGA1UdEwEB/wQCMAAwPAYDVR0fBDUwMzAxoC+gLYYrbW9z\n"\
"cXVpdHRvc2VydmVyLmxhbi9pbnRlcm1lZGlhdGUtY2EuY3JsLnBlbTARBglghkgB\n"\
"hvhCAQEEBAMCBaAwHQYDVR0lBBYwFAYIKwYBBQUHAwIGCCsGAQUFBwMEMAsGA1Ud\n"\
"DwQEAwIDqDAdBgNVHQ4EFgQUWeVBqGYkPEUAvmm/KL1M3dPZ0LcwHwYDVR0jBBgw\n"\
"FoAUTbUuu9YCfRfzW8IkFEccApguiRgwCgYIKoZIzj0EAwIDSAAwRQIgLE0f+dBM\n"\
"dVLzQ9ckIj2VrNNQzXdrfFWuJ95Dda+XrLcCIQCk0SZx3rUl1o6JI66/83M8HgCq\n"\
"dlnqlOwHAjfdmo7RXg==\n"\
"-----END CERTIFICATE-----"

#define client_key "-----BEGIN EC PARAMETERS-----\n"\
"BggqhkjOPQMBBw==\n"\
"-----END EC PARAMETERS-----\n"\
"-----BEGIN EC PRIVATE KEY-----\n"\
"MHcCAQEEIBeQUBufHAO+a/fWHdaT5gSIniF2AkwRyk56MQBrAaM0oAoGCCqGSM49\n"\
"AwEHoUQDQgAEPfz6Sw28WGrIKjX1rE55qKKQiqk9+MfHA9r8cowan9CvebwOlRoI\n"\
"fk9FI+0SRNT1mMI2MUxLnDBmIkd2ESQWXA==\n"\
"-----END EC PRIVATE KEY-----"

#define mqtt_ca_cert "-----BEGIN CERTIFICATE-----\n" \
"MIIBrjCCAVWgAwIBAgIRANyNugeNEoawITo4oewP6n8wCgYIKoZIzj0EAwIwNzEL\n" \
"MAkGA1UEBhMCQlIxFDASBgNVBAoMC05FUlNFQy1VRlNNMRIwEAYDVQQDDAlyb290\n" \
"LWxhbiAwHhcNMjUwNzE2MTcyNzMwWhcNMzUwNzE0MTcyNzMwWjA3MQswCQYDVQQG\n" \
"EwJCUjEUMBIGA1UECgwLTkVSU0VDLVVGU00xEjAQBgNVBAMMCXJvb3QtbGFuIDBZ\n" \
"MBMGByqGSM49AgEGCCqGSM49AwEHA0IABOsa+do/kpI+BA0uJacDJjx7Lqaxwct4\n" \
"Pw2MW7odHQ1grXbFRNZ3gw5QBek9sbdT/yJk+BF79UxoAfx82TP4D3OjQjBAMA8G\n" \
"A1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgEGMB0GA1UdDgQWBBSUnWdrM2mq\n" \
"e4olqtZL6mAtyaPjTjAKBggqhkjOPQQDAgNHADBEAiAqG8hvtpuf/NtfdNCojG+J\n" \
"WusMnqflX/iRe1MpxsCflQIgAu3gqFijzuqdo6PfwgEMq+wAF35I0D2QCq9EoZcW\n" \
"TNU=\n" \
"-----END CERTIFICATE-----\n" \
"-----BEGIN CERTIFICATE-----\n" \
"MIIC1zCCAn2gAwIBAgIRANyNugeNEoawITo4oewP6oAwCgYIKoZIzj0EAwIwNzEL\n" \
"MAkGA1UEBhMCQlIxFDASBgNVBAoMC05FUlNFQy1VRlNNMRIwEAYDVQQDDAlyb290\n" \
"LWxhbiAwHhcNMjUwNzE2MTcyNzMwWhcNMzUwNzE0MTcyNzMwWjA0MQswCQYDVQQG\n" \
"EwJCUjEUMBIGA1UECgwLTkVSU0VDLVVGU00xDzANBgNVBAMMBmNhLWxhbjBZMBMG\n" \
"ByqGSM49AgEGCCqGSM49AwEHA0IABLLAq9P0u2QI86kZxFs756lI9XsbeGsZRyEf\n" \
"Xi4P8fjf22ql3o5KwsdXKs1ivGOUTpUyhoerY5vCS6zptzuSxhyjggFrMIIBZzBk\n" \
"BggrBgEFBQcBAQRYMFYwKgYIKwYBBQUHMAKGHmh0dHA6Ly9yb290LWNhLmxhbi9y\n" \
"b290LWNhLmNydDAoBggrBgEFBQcwAYYcaHR0cDovL29jc3Aucm9vdC1jYS5sYW46\n" \
"OTg4MDAdBgNVHQ4EFgQUTbUuu9YCfRfzW8IkFEccApguiRgwEgYDVR0TAQH/BAgw\n" \
"BgEB/wIBADAvBgNVHR8EKDAmMCSgIqAghh5odHRwOi8vcm9vdC1jYS5sYW4vcm9v\n" \
"dC1jYS5jcmwwHQYDVR0lBBYwFAYIKwYBBQUHAwIGCCsGAQUFBwMBMA4GA1UdDwEB\n" \
"/wQEAwIBBjBLBgNVHR4ERDBCoA4wBYIDbGFuMAWCA2xhbqEwMAqHCAAAAAAAAAAA\n" \
"MCKHIAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAMB8GA1UdIwQYMBaA\n" \
"FJSdZ2szaap7iiWq1kvqYC3Jo+NOMAoGCCqGSM49BAMCA0gAMEUCIHMVTV9NWJtw\n" \
"wAO/5eTVL19PBnfToZQn8fIiZPPpE0VbAiEA8mhtRSAaKhZtAP29XSZZ9Ic2jJWx\n" \
"3Xw+Qezhi/j/SLU=\n" \
"-----END CERTIFICATE-----"


// ======= VARIÁVEIS DE CONTROLE DE RTOS (GLOBAL) ========
SemaphoreHandle_t wifiConectadoSemaphore;
SemaphoreHandle_t mqttConectadoSemaphore;
QueueHandle_t rfidDataQueue; // Fila para enviar EPCs lidos para a tarefa MQTT

// ======= FUNÇÃO AUXILIAR DE PUBLICAÇÃO MQTT ========
// Esta função é chamada PELA TAREFA 'TaskPublicarMQTT'
void publicarMQTT(const String& epc) {
  if (!mqttClient.connected()) {
    Serial.println("PublicarMQTT (auxiliar): Conexão MQTT perdida. Tentando reconectar para esta pub...");
    mqttClient.connect(CLIENT_ID); 
  }

  if (mqttClient.connected()) {
      JsonDocument doc;
      doc["epc"] = epc;
      doc["timestamp"] = String(millis());
      doc["mqttId"] = CLIENT_ID;

      String jsonString;
      serializeJson(doc, jsonString);

      mqttClient.publish(TOPICO_MQTT, jsonString.c_str());
      Serial.println("Publicado via MQTT: " + jsonString);
  } else {
      Serial.println("PublicarMQTT (auxiliar): Não foi possível publicar: Cliente não conectado.");
  }
}

// ======= TAREFA DE INICIALIZAÇÃO: CONEXÃO WI-FI ========
void TaskConectarWiFi(void *pvParameters) {
  Serial.print("Task ConectarWiFi: Iniciando conexão Wi-Fi...");
  WiFi.begin(SSID, SENHA);
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
  xSemaphoreGive(wifiConectadoSemaphore); // Sinaliza Wi-Fi conectado
  Serial.println("Task ConectarWiFi: Concluída e se auto-deletando.");
  vTaskDelete(NULL);
}

// ======= TAREFA DE INICIALIZAÇÃO: CONEXÃO MQTT ========
void TaskConectarMQTT(void *pvParameters) {
  Serial.println("Task ConectarMQTT: Aguardando conexão Wi-Fi...");
  xSemaphoreTake(wifiConectadoSemaphore, portMAX_DELAY); // Espera Wi-Fi conectado
  Serial.println("Task ConectarMQTT: Wi-Fi conectado, iniciando loop de conexão MQTT...");

  espClient.setCACert(mqtt_ca_cert);
  espClient.setCertificate(client_cert);
  espClient.setPrivateKey(client_key);
  mqttClient.setServer(BROKER_MQTT, PORTA_MQTT);

  // A TAREFA DEVE RODAR EM UM LOOP INFINITO PARA GERENCIAR A RECONEXÃO
  while (true) {
    if (!mqttClient.connected()) {
      Serial.print("Task ConectarMQTT: Tentando conectar ao broker MQTT...");
      // A PubSubClient precisa de um CLIENT_ID único por conexão.
      // É uma boa prática adicionar um timestamp ou um número aleatório
      // para garantir que seja único, especialmente após reconexões.
      String currentClientId = String(CLIENT_ID) + "_" + String(esp_random(), HEX);
      
      if (mqttClient.connect(currentClientId.c_str())) { // Use currentClientId aqui
        Serial.println(" [Task ConectarMQTT] Conectado ao MQTT!");
        // Opcional: Se houver algum tópico para se inscrever no cliente:
        // mqttClient.subscribe("seu/topico/sub");
        xSemaphoreGive(mqttConectadoSemaphore); // Sinaliza MQTT conectado
      } else {
        Serial.print(" [Task ConectarMQTT] Falha na conexão, rc=");
        Serial.print(mqttClient.state());
        Serial.println(". Tentando novamente em 5 segundos...");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Espera antes de tentar novamente
      }
    } else {
      // Se conectado, apenas cede tempo ou faz alguma manutenção leve.
      // O mqttClient.loop() principal é feito na TaskPublicarMQTT
      vTaskDelay(pdMS_TO_TICKS(100)); // Pequeno delay para ceder CPU
    }
  }
  // vTaskDelete(NULL); // NUNCA deve ser chamado aqui em uma tarefa de loop
}

// ======= TAREFA DE LEITURA RFID ========
void TaskLeituraRFID(void *pvParameters) {
    Serial.println("Task LeituraRFID: Iniciando.");

    // Configurações iniciais do RFID (apenas uma vez)
    rfid.begin(&Serial2, 115200, 16, 17); // Ajuste as portas Tx/Rx
    //rfid.dumpModuleInfo();
    rfid.setTransmissionPower(100);
    //rfid.acquireTransmitPower();

    // Atraso inicial para escalonamento
    unsigned long initial_delay_ms = T_SLOT_COMPLETO_MODULO * MEU_SLOT_INDEX;
    if (initial_delay_ms > 0) {
        Serial.printf("Task LeituraRFID: Delay inicial de escalonamento de %lu ms.\n", initial_delay_ms);
        vTaskDelay(pdMS_TO_TICKS(initial_delay_ms));
    }
    Serial.println("Task LeituraRFID: Escalonamento inicial concluído.");

    unsigned long ultimoPollNaJanela = 0;

    while (1) {
        // --- FASE ATIVA DE LEITURA PARA ESTE MÓDULO ---
        Serial.println("Task LeituraRFID: Iniciando Janela Ativa de Leitura...");
        unsigned long inicioJanelaAtiva = millis();
        ultimoPollNaJanela = inicioJanelaAtiva - T_INTERVALO_POLL_NA_JANELA; 

        while (millis() - inicioJanelaAtiva < T_JANELA_ATIVA_MODULO) {
            rfid.loop(); // Processa dados da serial do RFID

            if (millis() - ultimoPollNaJanela >= T_INTERVALO_POLL_NA_JANELA) {
                rfid.poll(); // Envia CMD_SinglePollInstruction
                ultimoPollNaJanela = millis();
            }

            rfid.loop(); // Processa resposta do poll
            if (rfid.epc.length() > 0) {
                Serial.print("Task LeituraRFID: TAG LIDA: "); Serial.println(rfid.epc);
                // Envia a tag para a fila da tarefa MQTT
                // É crucial que 'rfid.epc' seja um objeto String para ser copiado para a fila de String
                if (xQueueSend(rfidDataQueue, (void*)&rfid.epc, pdMS_TO_TICKS(10)) != pdPASS) {
                    Serial.println("Task LeituraRFID: Falha ao enviar TAG para a fila MQTT (fila cheia?).");
                }
                rfid.epc = ""; // Limpa após enviar para a fila
            }
            vTaskDelay(pdMS_TO_TICKS(1)); // Pequeno delay para ceder tempo
        }
        Serial.println("Task LeituraRFID: Fim da Janela Ativa de Leitura.");

        // Pequena pausa após a janela ativa
        vTaskDelay(pdMS_TO_TICKS(T_PAUSA_CURTA_POS_JANELA));

        // --- FASE INATIVA (ESPERA PELOS OUTROS MÓDULOS) ---
        Serial.printf("Task LeituraRFID: Iniciando Fase de Espera por %lu ms...\n", T_ESPERA_OUTROS_MODULOS);
        vTaskDelay(pdMS_TO_TICKS(T_ESPERA_OUTROS_MODULOS));
        Serial.println("Task LeituraRFID: Fim da Fase de Espera.");
    }
    vTaskDelete(NULL); // Não deve ser atingido em um loop infinito
}


// ======= TAREFA EM LOOP: PUBLICAR DADOS MQTT (Recebe da fila) ========
void TaskPublicarMQTT(void *pvParameters) {
    Serial.println("Task PublicarMQTT: Aguardando conexão MQTT...");
    // A primeira espera é para garantir que a conexão inicial esteja pronta
    xSemaphoreTake(mqttConectadoSemaphore, portMAX_DELAY); 

    Serial.println("Task PublicarMQTT: MQTT conectado. Iniciando publicações.");

    String epc_recebido; 

    while (true) { // Loop infinito
        // 1. Manter a conexão MQTT viva e processar eventos
        // Se a conexão cair, a TaskConectarMQTT deve reestabelecê-la
        if (mqttClient.connected()) {
            mqttClient.loop(); 
        } else {
            Serial.println("Task PublicarMQTT: MQTT desconectado. Aguardando reconexão...");
            // Espera até que a TaskConectarMQTT sinalize que o MQTT está novamente conectado
            // Isso pode bloquear por tempo indeterminado se TaskConectarMQTT falhar indefinidamente
            xSemaphoreTake(mqttConectadoSemaphore, portMAX_DELAY);
            Serial.println("Task PublicarMQTT: MQTT Reconectado (sinalizado).");
            continue; // Volta para o início do loop para garantir que loop() seja chamado antes de tentar enviar
        }

        // 2. Receber dados da fila (do RFID)
        // Se a fila estiver vazia, espera por 100ms e cede tempo.
        if (xQueueReceive(rfidDataQueue, (void*)&epc_recebido, pdMS_TO_TICKS(100))) {
            Serial.print("Task PublicarMQTT: Recebido da fila para publicar: "); Serial.println(epc_recebido);
            
            // Verifica a conexão antes de publicar. mqttClient.loop() já deve ter tentado manter.
            if (mqttClient.connected()) {
                JsonDocument doc;
                doc["epc"] = epc_recebido;
                doc["timestamp"] = String(millis());
                doc["mqttId"] = CLIENT_ID; // Use o CLIENT_ID definido globalmente

                String jsonString;
                serializeJson(doc, jsonString);

                if (mqttClient.publish(TOPICO_MQTT, jsonString.c_str())) {
                    Serial.println("Task PublicarMQTT: Publicado via MQTT: " + jsonString);
                } else {
                    Serial.print("Task PublicarMQTT: Falha ao publicar: Cliente conectado, mas publicacao falhou. rc=");
                    Serial.println(mqttClient.state());
                }
            } else {
                Serial.println("Task PublicarMQTT: Não foi possível publicar: Cliente não conectado (após tentativa de reconexão?).");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Ceder tempo
    }
}

// ======= SETUP ========
void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando: " __FILE__ " " __DATE__);

  // Inicialização do módulo RFID (agora no setup, pois rfid é global)
  rfid.begin(&Serial2, 115200, 16, 17);
  //rfid.dumpModuleInfo();
  rfid.setTransmissionPower(100);
  //rfid.acquireTransmitPower();

  // --- CRIAÇÃO DOS SEMÁFOROS ---
  wifiConectadoSemaphore = xSemaphoreCreateBinary();
  mqttConectadoSemaphore = xSemaphoreCreateBinary();
  
  if (wifiConectadoSemaphore == NULL || mqttConectadoSemaphore == NULL) {
    Serial.println("ERRO: Falha ao criar Semáforos! Sistema parado.");
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); } // Loop de erro fatal
  }

  // --- CRIAÇÃO DA FILA DE DADOS RFID ---
  // A fila pode conter N Strings. Capacidade: 5, Tamanho de cada item: sizeof(String)
  rfidDataQueue = xQueueCreate(5, sizeof(String));
  if (rfidDataQueue == NULL) {
      Serial.println("ERRO: Falha ao criar a fila de dados RFID! Sistema parado.");
      while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); } // Loop de erro fatal
  }

  // --- CRIAÇÃO DAS TAREFAS ---
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
    TaskLeituraRFID, // NOVA TAREFA: Leitura RFID
    "Leitura-RFID",
    4096, // Ajuste se necessário para sua lógica RFID
    NULL,
    3,    // Prioridade abaixo das de conexão, mas acima da publicação se desejar
    NULL,
    0     // Pode rodar em Core 0
  );

  xTaskCreatePinnedToCore(
    TaskPublicarMQTT, 
    "Publicar-MQTT",
    8192, // Tarefas de rede e loops podem precisar de mais stack
    NULL,
    2,    // Prioridade mais baixa, espera as outras
    NULL,
    0     // Pode rodar em Core 0, ou Core 1 se quiser aliviar Core 0
  );

  Serial.println("Todas as tarefas foram criadas pelo setup().");
  Serial.println("Sistema iniciado e escalonado (usando FreeRTOS tasks).");
}

// ======= LOOP ========
void loop() {
  // O loop principal do Arduino deve ser o mais leve possível quando usando FreeRTOS.
  // As tarefas FreeRTOS gerenciam a lógica.
  vTaskDelay(pdMS_TO_TICKS(1)); // Ceder tempo ao agendador.
}