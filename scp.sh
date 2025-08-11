#!/bin/sh

ESP_PORT="/dev/ttyUSB0" 

NVS_OFFSET="0x9000"
NVS_SIZE="0x4000"

NVS_BIN_FILE="res/nvs.bin"

NVS_DATA_CSV="res/nvs.csv"

python3 nvs_partition_gen.py --input "$NVS_DATA_CSV" --output "$NVS_BIN_FILE" --size $NVS_SIZE

python3 -m esptool --chip esp32 --port "$ESP_PORT" --baud 921600 \
  --before default_reset --after hard_reset \
  write_flash "$NVS_OFFSET" "$NVS_BIN_FILE"