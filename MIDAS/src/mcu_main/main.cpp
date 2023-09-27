#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

static constexpr size_t STACK_SIZE = 1024;

int i = 0;
/* Task to be created. */
void vTaskCode( void * pvParameters )
{
    /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. 
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );*/
    for( ;; )
    {
        /* Task code goes here. */
        Serial.println("Pigfarts here I come!");
        i++;
    }
}

void init_thread(TaskFunction_t function, const char* name, UBaseType_t priority, TaskHandle_t* handle) {
    BaseType_t xReturned;
    xReturned = xTaskCreate(function, name, STACK_SIZE, NULL, priority, handle);
    if (xReturned != pdPASS) {
        Serial.println("Failed to create task");
    }
}
void setup() {
    Serial.begin(9600);
}

void loop() {
}
