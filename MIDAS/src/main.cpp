#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

static constexpr size_t STACK_SIZE = 1024;
static unsigned char DATA_LOG_STACK[STACK_SIZE];
static unsigned char BAROMETER_STACK[STACK_SIZE];
static unsigned char LOW_G_STACK[STACK_SIZE];
static unsigned char HIGH_G_STACK[STACK_SIZE];
static unsigned char ORIENTATION_STACK[STACK_SIZE];
static unsigned char MAGNOMETER_STACK[STACK_SIZE];
static unsigned char GPS_STACK[STACK_SIZE];
static unsigned char GAS_STACK[STACK_SIZE];
static unsigned char VOLTAGE_STACK[STACK_SIZE];
static unsigned char CONTINUITY_STACK[STACK_SIZE];

constexpr std::size_t SENSOR_CORE = 0;
constexpr std::size_t DATA_CORE = 1;


/**
 * Reference to documentation of a type of function that could be a task
 * See below for actual, implemented functions
*/
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

/**
 *  Reference from documentation that creates non-core-pinned tasks
 *  Not currently in use as we are pinning all threads to cores
*/

void init_thread(TaskFunction_t function, const char* name, UBaseType_t priority, TaskHandle_t* handle) {
    BaseType_t xReturned;
    xReturned = xTaskCreate(function, name, STACK_SIZE, NULL, priority, handle);
    if (xReturned != pdPASS) {
        Serial.println("Failed to create task");
    }
}

/**
 * Thread functions, these are the code snippets that actually run upon initializing each respective thread
*/

void data_logger_thread(void* pvParameters) {
    while (true) {

    }
}

void barometer_thread(void* pvParameters) {
    while (true) {

    }
}

void low_g_thread(void* pvParameters) {
    while (true) {

    }
}

void high_g_thread(void* pvParameters) {
    while (true) {

    }
}

void orientation_thread(void* pvParameters) {
    while (true) {

    }
}

void magnometer_thread(void* pvParameters) {
    while (true) {

    }
}

void gps_thread(void* pvParameters) {
    while (true) {

    }
}

void gas_thread(void* pvParameters) {
    while (true) {

    }
}

void voltage_thread(void* pvParameters) {
    while (true) {

    }
}

void continuity_thread(void* pvParameters) {
    while (true) {

    }
}

/**
 * Creates all threads for each sensor, FSM, Kalman algorithim, and data logging member
*/
void setup() {
    StaticTask_t data_logger_task;
    xTaskCreateStaticPinnedToCore(data_logger_thread, "data_logger", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, DATA_LOG_STACK, &data_logger_task, SENSOR_CORE);

    StaticTask_t barometer_task;
    xTaskCreateStaticPinnedToCore(barometer_thread, "barometer", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, BAROMETER_STACK, &barometer_task, SENSOR_CORE);

    StaticTask_t low_g_task;
    xTaskCreateStaticPinnedToCore(low_g_thread, "low_g", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, LOW_G_STACK, &low_g_task, SENSOR_CORE);

    StaticTask_t high_g_task;
    xTaskCreateStaticPinnedToCore(high_g_thread, "hgih_g", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, HIGH_G_STACK, &high_g_task, SENSOR_CORE);

    StaticTask_t orientation_task;
    xTaskCreateStaticPinnedToCore(orientation_thread, "orientation", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, ORIENTATION_STACK, &orientation_task, SENSOR_CORE);

    StaticTask_t magnometer_task;
    xTaskCreateStaticPinnedToCore(magnometer_thread, "magnometer", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, MAGNOMETER_STACK, &magnometer_task, SENSOR_CORE);

    StaticTask_t gps_task;
    xTaskCreateStaticPinnedToCore(gps_thread, "gps", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, GPS_STACK, &gps_task, DATA_CORE);

    StaticTask_t gas_task;
    xTaskCreateStaticPinnedToCore(gas_thread, "gas", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, GAS_STACK, &gas_task, SENSOR_CORE);

    StaticTask_t voltage_task;
    xTaskCreateStaticPinnedToCore(voltage_thread, "voltage", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, VOLTAGE_STACK, &voltage_task, SENSOR_CORE);

    StaticTask_t continuity_task;
    xTaskCreateStaticPinnedToCore(continuity_thread, "continuity", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, CONTINUITY_STACK, &continuity_task, SENSOR_CORE);

    vTaskStartScheduler();
}

void loop() {

}
