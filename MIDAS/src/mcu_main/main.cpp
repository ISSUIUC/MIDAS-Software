#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

static const int STACK_SIZE = 1024;
void data_logger_thread(void* pvParameters) {
    configASSERT(((uint32_t) pvParameters ) == 1);
    while (true) {

    }
}

void setup() {
    TaskHandle_t data_logger_task = NULL;
    xTaskCreateStaticPinnedToCore(data_logger_thread, "data_logger", STACK_SIZE, NULL, tskIDLE_PRIORITY, &data_logger_task);
    vTaskStartScheduler();
}

void loop() {

}
