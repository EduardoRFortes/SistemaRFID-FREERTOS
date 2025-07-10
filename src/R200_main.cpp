#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "R200.h"

// ======= HEADERS DO FREERTOS PARA ARDUINO IDE ========
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ======= CONFIGURAÇÕES DE REDE E MQTT ========
const char* SSID = "nersec";
const char* SENHA = "gremio123";
const char* BROKER_MQTT = "172.22.48.50"; // Seu broker MQTT
const int PORTA_MQTT = 1883;

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
WiFiClient espClient;
PubSubClient mqttClient(espClient);
R200 rfid;

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
  xSemaphoreGive(wifiConectadoSemaphore); // Sinaliza Wi-Fi conectado
  Serial.println("Task ConectarWiFi: Concluída e se auto-deletando.");
  vTaskDelete(NULL);
}

// ======= TAREFA DE INICIALIZAÇÃO: CONEXÃO MQTT ========
void TaskConectarMQTT(void *pvParameters) {
  Serial.println("Task ConectarMQTT: Aguardando conexão Wi-Fi...");
  xSemaphoreTake(wifiConectadoSemaphore, portMAX_DELAY); // Espera Wi-Fi conectado
  Serial.println("Task ConectarMQTT: Wi-Fi conectado, iniciando conexão MQTT...");
  mqttClient.setServer(BROKER_MQTT, PORTA_MQTT);
  while (!mqttClient.connected()) {
    Serial.print(".");
    if (mqttClient.connect(CLIENT_ID)) {
      Serial.println("\nTask ConectarMQTT: Conectado ao MQTT!");
    } else {
      Serial.print(" Falha, rc=");
      Serial.print(mqttClient.state());
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  xSemaphoreGive(mqttConectadoSemaphore); // Sinaliza MQTT conectado
  Serial.println("Task ConectarMQTT: Concluída e se auto-deletando.");
  vTaskDelete(NULL);
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
    // Espera o sinal de que o MQTT está conectado antes de começar a publicar
    Serial.println("Task PublicarMQTT: Aguardando conexão MQTT...");
    xSemaphoreTake(mqttConectadoSemaphore, portMAX_DELAY); // Bloqueia indefinidamente até MQTT estar pronto

    Serial.println("Task PublicarMQTT: MQTT conectado. Iniciando publicações.");

    String epc_recebido; // Variável para receber a String da fila

    while (1) {
        // 1. Manter a conexão MQTT viva
        if (!mqttClient.connected()) {
            Serial.println("Task PublicarMQTT: Conexão MQTT perdida. Tentando reconectar...");
            mqttClient.connect(CLIENT_ID);
            if (!mqttClient.connected()) {
                vTaskDelay(pdMS_TO_TICKS(5000)); // Espera e tenta novamente
                continue;
            } else {
                Serial.println("Task PublicarMQTT: Reconectado ao MQTT.");
            }
        }
        mqttClient.loop(); // Processa mensagens MQTT e mantém a conexão

        // 2. Receber dados da fila (do RFID)
        // Se a fila estiver vazia, espera por 100ms e cede tempo.
        if (xQueueReceive(rfidDataQueue, (void*)&epc_recebido, pdMS_TO_TICKS(100))) {
            Serial.print("Task PublicarMQTT: Recebido da fila para publicar: "); Serial.println(epc_recebido);
            
            // 3. Chamar a função auxiliar para publicar o EPC recebido
            publicarMQTT(epc_recebido); 
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
  // As linhas comentadas abaixo são OK, mas verifique o funcionamento da sua biblioteca R200
  //rfid.dumpModuleInfo();
  rfid.setTransmissionPower(100);
  //rfid.acquireTransmitPower();

  // --- CRIAÇÃO DOS SEMÁFOROS ---
  wifiConectadoSemaphore = xSemaphoreCreateBinary();
  mqttConectadoSemaphore = xSemaphoreCreateBinary(); // CORREÇÃO AQUI: Cria o segundo semáforo
  
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
    TaskConectarWiFi, // Nome correto da função da tarefa
    "Conectar-WiFi",
    4096,
    NULL,
    5,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    TaskConectarMQTT, // Nome correto da função da tarefa
    "Conectar-MQTT",
    4096,
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
    TaskPublicarMQTT, // Nome correto da função da tarefa
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