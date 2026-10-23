#!/bin/sh

ESP_PORT="/dev/ttyUSB1" 

NVS_OFFSET="0x9000"
NVS_SIZE="0x4000"

NVS_BIN_FILE="res/nvs.bin"

NVS_DATA_CSV="res/nvs.csv"

python -m esp_idf_nvs_partition_gen generate  "$NVS_DATA_CSV" "$NVS_BIN_FILE"  $NVS_SIZE

python -m esptool --chip esp32 --port "$ESP_PORT" --baud 921600 \
  --before default_reset --after hard_reset \
  write_flash "$NVS_OFFSET" "$NVS_BIN_FILE"