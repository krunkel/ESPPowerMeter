PowerMeter
==========
(Arduino/PlatformIO/Visual Code)
ESP8266 based powermeter with following characteristics:
    - Primary powermeter using CS5460 module  (in my case used to measure total power consumption home)
    - 2 secundary current measurements (using current transformers) on ADS1115 module (in my case used to measure output 2 solar arrays)
    - 1 circuit to detect the zerocross/frequency of the voltage to calculate real power of secundary current measurements
    - integrated webserver (to be used in)
    - output is send to Nodered (and from nodered to InfluxDB and Grafana)

PowerMeter output will be used in my ESP32 based 'SmartCarCharge' project (A Smart Level 2 Car Charger)

