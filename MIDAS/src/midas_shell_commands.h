#include <systems.h>

MCommandExecutionResult set_fsm(const MShellContext& ctx) {
    // fix this command cause it's stupid
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;
    if(ctx.argc != 2) { return MCommandExecutionResult::ERR_INVAL_ARGC; }

    int st = atoi(ctx.argv[1]);
    arg->rocket_data.fsm_state.update((FSMState)st);
    return MCommandExecutionResult::OK;
}

MCommandExecutionResult hi_midas(const MShellContext& ctx) {
    Serial.print("hi ");
    // if no second argument, return
    if(ctx.argc != 2) { return MCommandExecutionResult::OK; }
    Serial.println(ctx.argv[1]);
    return MCommandExecutionResult::OK;
}

MCommandExecutionResult serial(const MShellContext& ctx){
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;
    // expecting 2 (get) or 3 (set) arguments
    if(ctx.argc < 2 || ctx.argc > 3) { return MCommandExecutionResult::ERR_INVAL_ARGC; }

    if(!strcmp(ctx.argv[1], "set")){
        if(ctx.argc != 3) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
        uint8_t serial = atoi(ctx.argv[2]);
        arg->eeprom.data.serial = serial;
        arg->eeprom.commit();
        Serial.printf("New Serial Number: %u", arg->eeprom.data.serial);
        return MCommandExecutionResult::OK;
    }
    else if(!strcmp(ctx.argv[1], "get")){
        Serial.printf("Serial Number: %u", arg->eeprom.data.serial);
        return MCommandExecutionResult::OK;
    }
    else{
        return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
    }
}

MCommandExecutionResult frequency(const MShellContext& ctx){
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;
    // expecting 2 (get) or 3 (set) arguments
    if(ctx.argc < 2 || ctx.argc > 3) { return MCommandExecutionResult::ERR_INVAL_ARGC; }

    if(!strcmp(ctx.argv[1], "set")){
        if(ctx.argc != 3) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
        float frequency = atoff(ctx.argv[2]);
        arg->eeprom.data.frequency = frequency;
        arg->eeprom.commit();
        Serial.printf("New Frequency: %u MHz", arg->eeprom.data.frequency);
        return MCommandExecutionResult::OK;
    }
    else if(!strcmp(ctx.argv[1], "get")){
        Serial.printf("Frequency: %u MHz", arg->eeprom.data.frequency);
        return MCommandExecutionResult::OK;
    }
    else{
        return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
    }
}

void m_shell_init_commands(MShell* sh) {
    sh->register_command("fsm", set_fsm, "fsm <state:int> - Sets the FSM state to state <state>");
    sh->register_command("hi", hi_midas, "hi <string> - Prints hi <string>");
    sh->register_command("serial", serial, "serial get - Get MIDAS serial number\nserial set <serialnumber:int> - Set MIDAS serial number");
    sh->register_command("frequency", frequency, "frequency get - Get MIDAS telemetry frequency (in MHz)\nfrequency set <freq:float> - Set MIDAS telemetry frequency (in MHz)");
}