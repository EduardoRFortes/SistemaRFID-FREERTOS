# Particionamento do ESP32 e Upload de Certificados com NVS

Este guia passo a passo detalha o procedimento para particionar a memória flash de um ESP32 e subir os certificados de segurança usando o NVS (Non-Volatile Storage).

## Passo 1: Configuração do platformio.ini

Abra o arquivo platformio.ini do seu projeto e adicione a seguinte linha para direcionar o PlatformIO ao seu arquivo de partição personalizado:

> board_build.partitions = res/partition.csv

## Passo 2: Mapeamento dos Certificados no nvs.csv

Edite o arquivo nvs.csv para mapear os caminhos dos seus certificados. Este arquivo é usado para definir os pares chave-valor que serão armazenados no NVS. Certifique-se de incluir os caminhos para o certificado do nó (Node.pem), a chave privada (Node.key) e a cadeia de certificados da CA (ca_chain.pem):

> key,type,encoding,value
> CERTS,namespace,,
> tarnodePUB,file,string,/etc/mosquitto/meus_certificados/nodes/tarnode1.1.lan.crt.pem
> tarnodePRIV,file,string,/etc/mosquitto/meus_certificados/nodes/tarnode1.1.lan.key
> tarnodeCHAIN,file,string,/etc/mosquitto/meus_certificados/ca_chain.crt.pem

## Passo 3: Permissões do Diretório de Certificados

Antes de prosseguir, garanta que o diretório que contém seus certificados (res/certs ou o caminho que você definiu) tenha as permissões de leitura adequadas. Isso é essencial para que o processo de upload funcione corretamente.

> sudo chmod +r /etc/mosquitto/meus_certificados/nodes/*

## Passo 4: Execução do Script de Upload

Com tudo configurado, execute o script scp.sh para carregar os certificados no ESP32. Este script se encarregará de usar as configurações do PlatformIO e o arquivo nvs.csv para particionar o ESP32 e enviar os arquivos de certificado para a partição NVS.

> ./scp.sh

Observação: Certifique-se de que o script scp.sh está configurado corretamente para o seu ambiente de desenvolvimento e para a porta serial do seu ESP32.

Este guia simplifica o processo, garantindo que os certificados sejam armazenados de forma segura no ESP32, prontos para serem utilizados em comunicações seguras (TLS/SSL).
