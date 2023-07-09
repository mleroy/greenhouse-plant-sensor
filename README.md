# greenhouse-plant-sensor
Code for a LilyGO T-Highgrow v1.1 plant sensor

Features:
- Low power consumption, can be powered from a 600mAh 3.7V Lithium-ion battery for approx. 3 months at the built-in 1hr measurement rate.
- Reads from all sensors (light, air temperature/humidity, soil moisture/salt, battery voltage)
- Uploads to an InfluxDB endpoint, including a runtime measurement (how long it took to connect to WiFi & take measurements before going to sleep)
- Wakes up based on a timer interrupt (during which all peripherals are turned off) 

To be built with Platform.IO (to properly resolve dependencies).

Based on https://github.com/Xinyuan-LilyGO/LilyGo-HiGrow
