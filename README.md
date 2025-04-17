# Greeonhouse_System

Greenhouse System Project

This project is designed to monitor and manage the environmental conditions of a greenhouse using the STM32F407VG microcontroller. The sensors used in the system measure various environmental parameters, and based on the data, components like fans, irrigation, and lights are controlled.

LDR (Light Dependent Resistor): Measures the light level, specifically targeting purple light. When the light level is low, the system turns on the lighting.

MQ135: Monitors air quality and CO2 levels. If the CO2 levels are high, it activates the fan to provide ventilation.

DHT11: Measures temperature and humidity. If the temperature or humidity is too high, the system activates the fan.

Soil Moisture Sensor: Measures soil moisture levels. If the moisture level is too low, it activates the irrigation pump.

These sensors allow the system to continuously monitor environmental conditions. When necessary, the system activates the fan, irrigation, and lighting to maintain optimal greenhouse conditions.

Sera Sistemi Projesi

Bu proje, STM32F407VG mikrodenetleyicisi ile sera koşullarını izleyen ve yöneten bir sistemdir. Sistemde kullanılan sensörler, ortamın çeşitli parametrelerini ölçer ve bu verilere dayalı olarak fan, sulama ve ışık gibi sistem bileşenlerini kontrol eder.

LDR (Işığa Bağımlı Direnç): Işık seviyelerini ölçer, özellikle mor ışığı hedef alır. Işık seviyesi düşük olduğunda, sistem ışıklandırmayı açar.

MQ135: Hava kalitesini ve CO2 seviyelerini izler. CO2 seviyeleri yüksek olduğunda, fanı çalıştırarak havalandırmayı sağlar.

DHT11: Sıcaklık ve nem verilerini ölçer. Sıcaklık veya nem seviyesi yüksekse, fanı çalıştırarak ortamı düzenler.

Toprak Nem Sensörü: Toprak nemini ölçer. Nem seviyesi düşükse, sulama pompası devreye girer.

Bu sensörler, sistemin çevresel koşulları sürekli olarak izlemesine olanak tanır. Gerekli durumlarda, fan, sulama ve ışık sistemlerini aktif hale getirerek sera ortamını kontrol eder.
