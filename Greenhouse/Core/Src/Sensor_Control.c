#include "Sensor_Control.h"

// Sensör nesneleri
Sensor soilMoistureSensor;
Sensor ldrSensor;
Sensor mq135Sensor;

extern TIM_HandleTypeDef htim2;
// Sensör başlatma fonksiyonu
void Sensor_Init(Sensor* sensor, uint32_t channel) {
    sensor->Channel = channel;
    sensor->Value = 0;  // Başlangıç değeri sıfır
}

// ADC kanalı okuma fonksiyonu
uint16_t Read_ADC_Channel(ADC_HandleTypeDef* hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;

    // ADC kanalını yapılandır
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        return 0; // Hata durumunda 0 döndür
    }

    // ADC dönüşüm işlemleri
    if (HAL_ADC_Start(hadc) != HAL_OK) {
        return 0;
    }
    if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return 0;
    }

    uint16_t value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return value;
}

// map fonksiyonunun tanımlanması
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// RGB LED kontrol fonksiyonu
void RGB_LED_Control(Sensor* sensor) {
    uint16_t adcValue = sensor->Value;

    // ADC değeri 0-4095 arasında
    uint8_t redValue = map(adcValue, 0, 4095, 0, 255);
    uint8_t greenValue = map(adcValue, 0, 4095, 0, 255);
    uint8_t blueValue = map(adcValue, 0, 4095, 0, 255);

    // PWM çıkışlarını ayarla
    __HAL_TIM_SET_COMPARE(&htim2, RED_PIN, redValue);
    __HAL_TIM_SET_COMPARE(&htim2, GREEN_PIN, greenValue);
    __HAL_TIM_SET_COMPARE(&htim2, BLUE_PIN, blueValue);
}

// Sensör değerlerini okuma fonksiyonu
void Sensor_Read(Sensor* sensor, ADC_HandleTypeDef* hadc) {
    sensor->Value = Read_ADC_Channel(hadc, sensor->Channel);
}


// Sistem başlatma fonksiyonu
void Sensor_System_Init(void) {
    // Sensörlerin yapılandırılması (kanal numaralarını ADC datasheet'e göre ayarla)
    Sensor_Init(&soilMoistureSensor, ADC_CHANNEL_1); // Toprak Nemi
    Sensor_Init(&ldrSensor, ADC_CHANNEL_2);          // LDR
    Sensor_Init(&mq135Sensor, ADC_CHANNEL_3);        // MQ135
}

void Sensor_System_Update(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, ADC_HandleTypeDef* hadc3) {
    // Sensör değerlerini sırayla oku
    Sensor_Read(&soilMoistureSensor, hadc1); // ADC1'den oku
    Sensor_Read(&ldrSensor, hadc2);          // ADC2'den oku
    Sensor_Read(&mq135Sensor, hadc3);        // ADC3'ten oku

    // RGB LED'i kontrol et
    RGB_LED_Control(&soilMoistureSensor);
    RGB_LED_Control(&ldrSensor);
    RGB_LED_Control(&mq135Sensor);
}


// ADC değerini yüzdeye dönüştüren genel fonksiyon LDR VE Toprak Nem Algılama Sensörü İçin
uint8_t ADC_To_Percentage(Sensor* sensor) {
    // ADC değeri 0-4095 arasında
    uint8_t percentage = (sensor->Value * 100) / 4095;
    return percentage;
}


