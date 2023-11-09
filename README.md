# Transmeter module for TVC rocket

### Components
- `GY87`/`GY88` Module
- Long Distance Comunication Module
    - ESP-NOW
    - Exploited Brodcast mode
    - LoRA Module
---

### Used Device 
- ESP32 DevKit V1
- ESp32S-CAM 
- GY87 Module
- 

---
### Configuration
- Freq : `240Hz`
- Flash : `4Mb`
- I2C_SCL : `GPIO_22`
- I2C_SDA : `GPIO_21`
- I2C_Freq : `100000`

---
### How to run ? 
- activate idf environment
- `get-idf` (create alise for `. ~/esp/esp-idf/export.sh`)
- `idf.py build`
- `idf.py -p /dev/ttyUSB0 flash monitor`
