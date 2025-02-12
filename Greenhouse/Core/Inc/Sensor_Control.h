#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H

#include "stm32f4xx_hal.h"
#include "math.h"

// Sensör yapı tanımı
typedef struct {
    uint16_t Value;            // Sensör ADC değeri
    uint32_t Channel;          // ADC kanalı
} Sensor;

// Sensörler (dışarıdan erişilebilir global değişkenler)
extern Sensor soilMoistureSensor;  // Toprak Nemi Algılama Sensörü
extern Sensor ldrSensor;           // LDR
extern Sensor mq135Sensor;         // MQ135
// RGB LED pinleri
#define RED_PIN TIM_CHANNEL_1
#define GREEN_PIN TIM_CHANNEL_2
#define BLUE_PIN TIM_CHANNEL_3


// Fonksiyon Prototipleri
void Sensor_Init(Sensor* sensor, uint32_t channel);
uint16_t Read_ADC_Channel(ADC_HandleTypeDef* hadc, uint32_t channel);
void Sensor_Read(Sensor* sensor, ADC_HandleTypeDef* hadc);
void Sensor_System_Init(void); // Tüm sensörleri başlatır
void Sensor_System_Update(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, ADC_HandleTypeDef* hadc3); // Tüm sensörleri günceller
uint8_t ADC_To_Percentage(Sensor* sensor); // HAM ADC değerlerini yüzdeliğe dönüştürür
uint8_t calculate_CO2_concentration(uint16_t adc_value); // CO2 konsantrasyonu hesaplama fonksiyonu prototipi
uint8_t read_mq135(Sensor* sensor); // MQ-135 sensörünü okuma fonksiyonu prototipi


#endif // SENSOR_CONTROL_H
