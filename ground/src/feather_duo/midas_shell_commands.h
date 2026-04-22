#include "duo.h"
#include <string>

void print_serial(uint8_t serial){
    uint8_t str [3];
    if (serial < 10){
        Serial.print("00");
        Serial.println(serial);
        return;
    }
    if (serial < 100){
        Serial.print("0");
        Serial.println(serial);
        return;
    }
    Serial.println(serial);
}

MCommandExecutionResult hi_feather(const MShellContext& ctx) {
    Serial.print("hi ");
    // if no second argument, return
    if(ctx.argc != 2) { return MCommandExecutionResult::OK; }
    Serial.println(ctx.argv[1]);
    return MCommandExecutionResult::OK;
}

MCommandExecutionResult serial(const MShellContext& ctx){
    DuoSystems* arg = (DuoSystems*) ctx.sysarg;
    // expecting 3 (get) or 4 (set) arguments
    if(ctx.argc < 3 || ctx.argc > 4) { return MCommandExecutionResult::ERR_INVAL_ARGC; }

    // check for valid radio key
    uint8_t key = atoi(ctx.argv[1]);
    if(key != 0 && key != 1) { return MCommandExecutionResult::ERR_INVAL_ARGUMENT; } 

    if(!strcmp(ctx.argv[2], "set")){
        if(ctx.argc != 3) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
        uint8_t serial = atoi(ctx.argv[3]);
        arg->eeprom.data.serial[key] = serial;
        arg->eeprom.commit();
        Serial.print("New Serial Number: ");
        print_serial(arg->eeprom.data.serial[key]);
        return MCommandExecutionResult::OK;
    }
    else if(!strcmp(ctx.argv[2], "get")){
        print_serial(arg->eeprom.data.serial[key]);
        return MCommandExecutionResult::OK;
    }
    else{
        return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
    }
}

MCommandExecutionResult frequency(const MShellContext& ctx){
    DuoSystems* arg = (DuoSystems*) ctx.sysarg;
    // expecting 3 (get) or 4 (set) arguments
    if(ctx.argc < 3 || ctx.argc > 4) { return MCommandExecutionResult::ERR_INVAL_ARGC; }

    // check for valid radio key
    uint8_t key = atoi(ctx.argv[1]);
    if(key != 0 && key != 1) { return MCommandExecutionResult::ERR_INVAL_ARGUMENT; } 

    if(!strcmp(ctx.argv[2], "set")){
        if(ctx.argc != 3) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
        float frequency = atoff(ctx.argv[3]);
        // ensure that given frequency is within the 70cm band 
        if (frequency > 450.0 || frequency < 420.0) { return MCommandExecutionResult::ERR_INVAL_ARG_RANGE; }
        
        // set eeprom frequency
        arg->eeprom.data.frequency[key] = frequency;
        arg->eeprom.commit();

        // set telemetry frequency
        arg->cfg[key].desired_frequency = frequency;

        Serial.print("New Frequency: ");
        Serial.print(arg->eeprom.data.frequency[key]);
        Serial.println("MHz");
        return MCommandExecutionResult::OK;
    }
    else if(!strcmp(ctx.argv[2], "get")){
        Serial.print(arg->eeprom.data.frequency[key]);
        Serial.println(" MHz");
        return MCommandExecutionResult::OK;
    }
    else{
        return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
    }
}


int8_t checkSerial(DuoSystems* sys, uint8_t serial){
    if (serial == sys->eeprom.data.serial[0]){
        return 0;
    }
    else if (serial == sys->eeprom.data.serial[1]){
        return 1;
    }
    return -1;
}


MCommandExecutionResult pyro(const MShellContext& ctx){
    DuoSystems* arg = (DuoSystems*) ctx.sysarg;
    // expecting 3 (test) or 4 (fire) arguments
    if(ctx.argc < 3 || ctx.argc > 4) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    uint8_t serial = atoi(ctx.argv[1]);
    int8_t key = checkSerial(arg, serial);
    if (key < 0) { return MCommandExecutionResult::ERR_INVAL_SERIAL; }

    TelemetryCommand command{};

    if (!strcmp(ctx.argv[2], "test")){
        command.command = CommandType::SWITCH_TO_PYRO_TEST;
    }
    else if (!strcmp(ctx.argv[2], "fire")){
        if (!strcmp(ctx.argv[3], "A")){
            command.command = CommandType::FIRE_PYRO_A;
        }
        else if (!strcmp(ctx.argv[3], "B")){
            command.command = CommandType::FIRE_PYRO_B;
        }
        else if (!strcmp(ctx.argv[3], "C")){
            command.command = CommandType::FIRE_PYRO_C;
        }
        else if (!strcmp(ctx.argv[3], "D")){
            command.command = CommandType::FIRE_PYRO_D;
        }
        else{
            return MCommandExecutionResult::ERR_INVALID_CMD;
        }
    }
    else{
        return MCommandExecutionResult::ERR_INVALID_CMD;
    }

    arg->cfg[key].cmd_queue->send(command);

    return MCommandExecutionResult::OK;
}


MCommandExecutionResult state(const MShellContext& ctx){
    DuoSystems* arg = (DuoSystems*) ctx.sysarg;
    // expecting 3 arguments
    if(ctx.argc == 3) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    uint8_t serial = atoi(ctx.argv[1]);
    int8_t key = checkSerial(arg, serial);
    if (key < 0) { return MCommandExecutionResult::ERR_INVAL_SERIAL; }

    TelemetryCommand command{};

    if (!strcmp(ctx.argv[2], "SAFE")){
        command.command = CommandType::SWITCH_TO_SAFE;
    }
    else if (!strcmp(ctx.argv[2], "ARMED")){
        command.command = CommandType::SWITCH_TO_ARMED;
    }
    else{
        return MCommandExecutionResult::ERR_INVALID_CMD;
    }

    arg->cfg[key].cmd_queue->send(command);

    return MCommandExecutionResult::OK;
}


MCommandExecutionResult cam(const MShellContext& ctx){
    DuoSystems* arg = (DuoSystems*) ctx.sysarg;
    // expecting 3 arguments
    if(ctx.argc == 3) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    uint8_t serial = atoi(ctx.argv[1]);
    int8_t key = checkSerial(arg, serial);
    if (key < 0) { return MCommandExecutionResult::ERR_INVAL_SERIAL; }

    TelemetryCommand command{};

    if (!strcmp(ctx.argv[2], "on")){
        command.command = CommandType::CAM_ON;
    }
    else if (!strcmp(ctx.argv[2], "off")){
        command.command = CommandType::CAM_OFF;
    }
    else if (!strcmp(ctx.argv[2], "toggle")){
        command.command = CommandType::TOGGLE_CAM_VMUX;
    }
    else{
        return MCommandExecutionResult::ERR_INVALID_CMD;
    }

    arg->cfg[key].cmd_queue->send(command);

    return MCommandExecutionResult::OK;
}

MCommandExecutionResult calib(const MShellContext& ctx){
    DuoSystems* arg = (DuoSystems*) ctx.sysarg;
    // expecting 3 arguments
    if(ctx.argc == 3) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    uint8_t serial = atoi(ctx.argv[1]);
    int8_t key = checkSerial(arg, serial);
    if (key < 0) { return MCommandExecutionResult::ERR_INVAL_SERIAL; }

    TelemetryCommand command{};

    if (!strcmp(ctx.argv[2], "xl") || !strcmp(ctx.argv[2], "accel") || !strcmp(ctx.argv[2], "accelerometer")){
        command.command = CommandType::CALIB_ACCEL;
    }
    else if (!strcmp(ctx.argv[2], "mag") || !strcmp(ctx.argv[2], "magnetometer")){
        command.command = CommandType::CALIB_MAG;
    }
    else{
        return MCommandExecutionResult::ERR_INVALID_CMD;
    }

    arg->cfg[key].cmd_queue->send(command);

    return MCommandExecutionResult::OK;
}


void m_shell_init_commands(MShell* sh) {
    sh->register_command("hi", hi_feather, "\t\thi <string> - Prints hi <string>");
    sh->register_command("serial", serial, "\tserial (X) get - Get MIDAS serial number associated with radio X\n\t\tserial (X) set <serialnumber:int> - Set MIDAS serial number associated with radio X");
    sh->register_command("frequency", frequency, "\tfrequency (X) get - Get radio X frequency (in MHz)\n\t\tfrequency (X) set <freq:float> - Set radio X frequency (in MHz)");
    sh->register_command("pyro", pyro, "\tpyro (serial) fire [A|B|C|D] - Fire pyro channel if in pyro test state\n\t\tpyro (serial) test - Put MIDAS into pyro test state");
    sh->register_command("state", state, "\tstate (serial) [SAFE|ARMED] - Set MIDAS FSM state");
    sh->register_command("cam", cam, "\tcam (serial) [on|off|toggle] - Send commands to CAM via MIDAS");
    sh->register_command("calib", calib, "\tcalib (serial) [mag|magnetometer|xl|accel|accelerometer] - Calibrate MIDAS sensors");
}