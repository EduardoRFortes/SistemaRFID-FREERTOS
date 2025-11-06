#include "RFIDR200.h"

// Constructor para HardwareSerial
// O '&' depois de HardwareSerial significa que estamos passando uma referência ao objeto.
RFIDR200::RFIDR200(HardwareSerial &serial, uint32_t baudRate) : serial(serial), baudRate(baudRate) {}

// O construtor de SoftwareSerial foi removido.

void RFIDR200::begin() {
    // A chamada para serial.begin() foi REMOVIDA daqui.
    // Isso é MUITO IMPORTANTE. A inicialização da porta serial, especialmente
    // a definição dos pinos RX/TX no ESP32, DEVE ser feita no arquivo principal (main.cpp).
    // Esta função pode ser usada no futuro para outros comandos de inicialização do módulo.
}

// Função para definir os parâmetros avançados do protocolo Gen2 (Query e Session)
// startQ: Valor Q inicial (0-15)
// minQ: Valor Q mínimo para o algoritmo dinâmico (0-15)
// maxQ: Valor Q máximo para o algoritmo dinâmico (0-15)
// session: A sessão de inventário (S0, S1, S2, S3)
bool RFIDR200::setQueryParameters(uint8_t startQ, uint8_t minQ, uint8_t maxQ, uint8_t session) {

    uint8_t command[] = {
        0xAA, 0x00, 0x0E, 0x00, 0x02, // Cabeçalho, Código do Comando, Tamanho do payload
        0x00, // Byte para os parâmetros 1 (conterá startQ e session)
        0x00, // Byte para os parâmetros 2 (conterá minQ e maxQ)
        0x00, // Espaço para o checksum
        0xDD  // Fim do comando
    };

    // Esta parte "empacota" os seus parâmetros em 2 bytes, usando operações de bits.
    // Byte de Parâmetro 1: startQ nos 4 bits de cima, Session nos 2 bits de baixo.
    command[5] = (startQ << 4) | (session & 0x03);
    // Byte de Parâmetro 2: maxQ nos 4 bits de cima, minQ nos 4 bits de baixo.
    command[6] = (maxQ << 4) | (minQ & 0x0F);

    // Calcula o checksum e insere no pacote
    command[7] = calculateChecksum(command, 8);
    
    // Limpa o buffer e envia o comando
    while(serial.available()) serial.read();
    sendCommand(command, sizeof(command));

    // Espera e verifica a resposta de sucesso
    uint8_t responseBuffer[10];
    if (getResponse(responseBuffer, sizeof(responseBuffer))) {
        // Verifica se a resposta corresponde ao comando enviado e se o status é sucesso (0x00)
        if (responseBuffer[2] == 0x0E && responseBuffer[5] == 0x00) {
            return true; // Sucesso!
        }
    }
    
    // Se não houver resposta ou se a resposta indicar erro, retorna falso.
    return false;
}

bool RFIDR200::setTransmitPower(uint16_t power) {
    uint8_t command[] = {
        0xAA, 0x00, 0xB6, 0x00, 0x02, (uint8_t)(power >> 8), (uint8_t)(power & 0xFF), 0x00, 0xDD
    };
    command[7] = calculateChecksum(command, 8);
    
    // Limpa o buffer de receção antes de enviar para garantir que lemos a resposta correta.
    while(serial.available()) serial.read();

    sendCommand(command, sizeof(command));

    // Espera por uma resposta do módulo.
    uint8_t responseBuffer[10];
    if (getResponse(responseBuffer, sizeof(responseBuffer))) {
        // Verifica se a resposta corresponde ao comando enviado (0xB6) e se o status é sucesso (0x00).
        if (responseBuffer[2] == 0xB6 && responseBuffer[5] == 0x00) {
            return true; // Sucesso!
        }
    }

    // Se não houver resposta (timeout) ou a resposta indicar um erro, retorna falso.
    return false;
}


void RFIDR200::initiateMultiplePolling(uint16_t count) {
    uint8_t command[] = {0xAA, 0x00, 0x27, 0x00, 0x03, 0x22, (uint8_t)(count >> 8), (uint8_t)(count & 0xFF), 0x00, 0xDD};
    command[8] = calculateChecksum(command, 9); // Checksum é calculado sobre 9 bytes
    sendCommand(command, sizeof(command));
}

// Dentro de RFIDR200.cpp:
// ... (outras funções)

// Função que espera a resposta do módulo (É A FUNÇÃO MAIS CRÍTICA!)
bool RFIDR200::getResponse(uint8_t *buffer, size_t length, uint32_t timeout) {
    uint32_t startTime = millis();
    size_t bufferIndex = 0;
    int frameLength = -1;
    
    // O loop espera pelo frame completo
    while ((millis() - startTime) < timeout) {
        if (serial.available()) {
            uint8_t byteRead = serial.read();
            
            if (bufferIndex == 0 && byteRead != 0xAA) {
              // Espera pelo Header (0xAA)
              continue; 
            }

            if (bufferIndex < length) {
                buffer[bufferIndex++] = byteRead;
            } else {
                // Buffer cheio, descarta o byte
                continue;
            }

            if(bufferIndex >= 5 && frameLength == -1) {
              // Calcula o tamanho total do frame
              int payloadLength = (buffer[3] << 8) | buffer[4];
              frameLength = 5 + payloadLength + 2; // Cabeçalho + Payload + Checksum + Fim (DD)
            }

            if(frameLength != -1 && bufferIndex >= (size_t)frameLength) {
                return true; // Frame completo recebido
            }
        }
        
        // CORREÇÃO CRÍTICA: Cede o controle ao FreeRTOS para evitar o Watchdog.
        // Se a CPU esperar muito tempo aqui, ela reinicia.
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
    return false;  // Timeout
}

// ... (o resto do seu RFIDR200.cpp)


// --- O RESTANTE DO SEU CÓDIGO .CPP PERMANECE O MESMO ---
// (As funções abaixo não precisam de alteração)

void RFIDR200::readTagData(uint32_t accessPassword, uint8_t memBank, uint16_t address, uint16_t length, uint8_t *data) {
    uint8_t command[] = {
        0xAA, 0x00, 0x39, 0x00, 0x09, 
        (uint8_t)(accessPassword >> 24), (uint8_t)(accessPassword >> 16), 
        (uint8_t)(accessPassword >> 8), (uint8_t)(accessPassword & 0xFF), 
        memBank, 
        (uint8_t)(address >> 8), (uint8_t)(address & 0xFF), 
        (uint8_t)(length >> 8), (uint8_t)(length & 0xFF), 
        0x00, 0xDD
    };
    command[14] = calculateChecksum(command, 15);
    sendCommand(command, sizeof(command));
    getResponse(data, length * 2 + 10);
}

void RFIDR200::writeTagData(uint32_t accessPassword, uint8_t memBank, uint16_t address, uint16_t length, uint8_t *data) {
    uint8_t command[18 + length * 2] = {
        0xAA, 0x00, 0x49, 0x00, static_cast<uint8_t>(9 + length * 2),
        (uint8_t)(accessPassword >> 24), (uint8_t)(accessPassword >> 16), 
        (uint8_t)(accessPassword >> 8), (uint8_t)(accessPassword & 0xFF), 
        memBank, 
        (uint8_t)(address >> 8), (uint8_t)(address & 0xFF), 
        (uint8_t)(length >> 8), (uint8_t)(length & 0xFF)
    };
    for (uint16_t i = 0; i < length * 2; i++) {
        command[14 + i] = data[i];
    }
    command[14 + length * 2] = calculateChecksum(command, 15 + length * 2);
    command[15 + length * 2] = 0xDD;
    sendCommand(command, sizeof(command));
}


// Função para configurar os parâmetros do recetor (sensibilidade vs. velocidade)
bool RFIDR200::setDemodulatorParameters(uint8_t mixerGain, uint8_t ifGain, uint16_t threshold) {
    uint8_t command[] = {
        0xAA, 0x00, 0xF0, 0x00, 0x04, // Cabeçalho, Comando, Tamanho do Payload
        mixerGain,                    // Parâmetro 1: Mixer Gain
        ifGain,                       // Parâmetro 2: IF Gain
        (uint8_t)(threshold >> 8),    // Parâmetro 3 (parte alta): Threshold MSB
        (uint8_t)(threshold & 0xFF),  // Parâmetro 3 (parte baixa): Threshold LSB
        0x00,                         // Espaço para o checksum
        0xDD                          // Fim
    };

    // O checksum agora é na posição 9
    command[9] = calculateChecksum(command, 10);

    while(serial.available()) serial.read();
    sendCommand(command, sizeof(command));

    uint8_t responseBuffer[10];
    if (getResponse(responseBuffer, sizeof(responseBuffer))) {
        // A resposta de sucesso para o comando 0xF0 é (FrameType 0x01, Comando 0xF0, Status 0x00)
        // Na nossa biblioteca, isto corresponde a verificar o comando (posição 2) e o status (posição 5)
        if (responseBuffer[2] == 0xF0 && responseBuffer[5] == 0x00) {
            return true; // Sucesso!
        }
    }
    
    return false; // Falha
}

void RFIDR200::initiateSinglePolling() {
    uint8_t command[] = {0xAA, 0x00, 0x22, 0x00, 0x00, 0x22, 0xDD};
    sendCommand(command, sizeof(command));
}

void RFIDR200::stopMultiplePolling() {
    uint8_t command[] = {0xAA, 0x00, 0x28, 0x00, 0x00, 0x28, 0xDD};
    sendCommand(command, sizeof(command));
}

void RFIDR200::parseTagResponse(uint8_t* response, uint8_t& rssi, uint8_t (&epc)[12]) {
    rssi = response[5];
    memcpy(epc, &response[8], 12);
}

void RFIDR200::sendCommand(uint8_t *command, size_t length) {
    serial.write(command, length);
}

uint8_t RFIDR200::calculateChecksum(uint8_t *command, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < length - 1; i++) { // Checksum calculation usually excludes start and end bytes
        checksum += command[i];
    }
    return checksum & 0xFF;
}

bool RFIDR200::hasValidTag(uint8_t *response) {
    if (response[2] == 0xFF) {
      checkErrorCode(response[5]);
      return false;
    }
   return true;
}

int RFIDR200::checkErrorCode(uint8_t code) {
    switch (code) {
        case 0x17: Serial.println("Error: Command code error."); return 2;
        case 0x20: Serial.println("Error: FHSS Fail."); return 2;
        case 0x15: /*Serial.println("Info: Inventory Fail (No tag found).");*/ return 1;
        case 0x16: Serial.println("Error: Access Fail."); return 2;
        case 0x09: Serial.println("Error: Read Fail."); return 2;
        case 0xA0: Serial.println("Error: Read Error."); return 2;
        case 0x10: Serial.println("Error: Write Fail."); return 2;
        case 0xB0: Serial.println("Error: Write Error."); return 2;
        case 0x13: Serial.println("Error: Lock Fail."); return 2;
        case 0xC0: Serial.println("Error: Lock Error."); return 2;
        case 0x12: Serial.println("Error: Kill Fail."); return 2;
        default: Serial.print("Error: Unknown Code: 0x"); Serial.println(code, HEX); return 2;
    }
    return 0;
}
// ... (O resto das suas funções como killTag, lockTag, etc. podem ficar aqui sem alterações)

