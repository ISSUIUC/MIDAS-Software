#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

static const int STACK_SIZE = 1024;
constexpr std::size_t SENSOR_CORE = 0;
constexpr std::size_t DATA_CORE = 1;

void data_logger_thread(void* pvParameters) {
    configASSERT(((uint32_t) pvParameters ) == 1);
    while (true) {

    }
}

void altitude_thread(void* pvParameters) {
    configASSERT(((uint32_t) pvParameters ) == 1);
    while (true) {

    }
}

void setup() {
    TaskHandle_t data_logger_task = NULL;
    xTaskCreateStaticPinnedToCore(data_logger_thread, "data_logger", STACK_SIZE, NULL, tskIDLE_PRIORITY, &data_logger_task);

    TaskHandle_t altitude_task = NULL;
    xTaskCreateStaticPinnedToCore(altitude_thread, "altitude", STACK_SIZE, NULL, tskIDLE_PRIORITY+1, &altitude_task);
    vTaskStartScheduler();
}

void loop() {

}
