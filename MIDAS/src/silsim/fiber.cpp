#include "fiber.h"

constexpr size_t REAL_STACK_SIZE = 8372224;

#if defined(WIN32) || defined(_WIN32) || defined(_WIN64) || defined(__MINGW32__)
#include <windows.h>

void EmuSwitchToFiber(FiberHandle handle) { SwitchToFiber(handle.handle); }
FiberHandle EmuCreateFiber(size_t stack_size, ThreadFunc func, void* arg) {
    void* handle = CreateFiber(REAL_STACK_SIZE, func, arg);
    return {.handle = handle, .emu_stack_size = stack_size, .is_main = false};
}

FiberHandle EmuConvertThreadToFiber() {
    void* handle = ConvertThreadToFiber(nullptr);
    return {.handle = handle, .emu_stack_size = 0, .is_main = true};
}
#else
#ifdef __APPLE__
#define _XOPEN_SOURCE
#endif

#include <ucontext.h>

#include <cassert>
#include <iostream>
#include <string.h>
#include <vector>
#include <utility>

typedef void VoidFunc(void);

constexpr const char* stack_end = "STACKEND";
static ucontext_t main_context;
static ucontext_t* current_context;
static std::vector<std::pair<void*, void*>> threadInfoStore;

void thread_wrapper(int idx){
    auto [func, arg] = threadInfoStore.at(idx);
    ((ThreadFunc*)func)(arg);
}

void EmuSwitchToFiber(FiberHandle handle) {
    ucontext_t* to_context = current_context;
    current_context = handle.handle;
    swapcontext(to_context, handle.handle);
}

FiberHandle EmuCreateFiber(size_t stack_size, ThreadFunc func, void* arg) {
    ucontext_t* context = new ucontext_t{};
    if (getcontext(context) == -1) {
        assert(!"Get context fail");
    }

    char* stack = new char[REAL_STACK_SIZE];
    context->uc_stack.ss_sp = stack;
    context->uc_stack.ss_size = REAL_STACK_SIZE;
    context->uc_link = &main_context;
    threadInfoStore.push_back({(void*)func, arg});
    makecontext(context, (VoidFunc*)thread_wrapper, 1, threadInfoStore.size()-1);
    return {.handle = context, .emu_stack_size = stack_size, .is_main = false};
}

// always main thread
FiberHandle EmuConvertThreadToFiber() {
    current_context = &main_context;
    return {.handle = &main_context, .emu_stack_size = 0, .is_main = true};
}
#endif