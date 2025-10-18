#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configTICK_RATE_HZ 1000
#define configUSE_PREEMPTION 0
#define configUSE_MUTEXES 1
#define configSUPPORT_STATIC_ALLOCATION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 0
#define configKERNAL_PROVIDED_STATIC_MEMORY 1
#define INCLUDE_vTaskDelay 1

// #define configCHECK_FOR_STACK_OVERFLOW 2

#define configMINIMAL_STACK_SIZE 8192
#define configMAX_PRIORITIES 16
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_16_BIT_TICKS 0

#endif