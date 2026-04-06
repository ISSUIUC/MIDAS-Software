#include <systems.h>


FSMConfiguration shell_cfg;

enum class DataType {
    FLOAT, BOOL, UINT8, FSMSTATE
};

struct MapEntry {
    const char* name;
    size_t offset;
    DataType type;
    const char* unit;
};

static constexpr MapEntry threshold_map[] = {
    {"PYRO_FIRE_T", offsetof(FSMUserThresholds, pyro_fire_t), DataType::FLOAT, "ms"},
    {"MAIN_ALT", offsetof(FSMUserThresholds, main_alt), DataType::FLOAT, "m"},
    {"CRUISE_LOCKOUT_EN", offsetof(FSMUserThresholds, cruise_lockout_en), DataType::BOOL, ""},
};

static constexpr MapEntry channel_map[] = {
    {"ENABLE", offsetof(FSMPyroAction, enable), DataType::BOOL, ""},
    {"FSM_TRIGGER", offsetof(FSMPyroAction, fsm_trigger), DataType::FSMSTATE, ""},
    {"DELAY", offsetof(FSMPyroAction, delay), DataType::FLOAT, "ms"},
    {"MAX_TILT", offsetof(FSMPyroAction, max_tilt), DataType::FLOAT, "degrees"},
    {"AFTER_MOTOR", offsetof(FSMPyroAction, after_motor), DataType::UINT8, ""},
    {"LAUNCH_T_GT", offsetof(FSMPyroAction, launch_t_gt), DataType::FLOAT, "ms"},
    {"LAUNCH_T_LT", offsetof(FSMPyroAction, launch_t_lt), DataType::FLOAT, "ms"},
    {"VX_MIN", offsetof(FSMPyroAction, vx_min), DataType::FLOAT, "m/s"},
    {"VX_MAX", offsetof(FSMPyroAction, vx_max), DataType::FLOAT, "m/s"},
};

static constexpr const char * state_names[] = {
    "SAFE",
    "PYRO_TEST",
    "ARMED",
    "BOOST",
    "COAST",
    "APOGEE",
    "DROGUE",
    "MAIN",
    "LANDED",
};

static constexpr FSMState allowed_trigger_states[] = {
    FSMState::STATE_COAST, FSMState::STATE_DROGUE, FSMState::STATE_MAIN
};

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

MCommandExecutionResult set_fsm(const MShellContext& ctx) {
    // fix this command cause it's stupid
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;
    if(ctx.argc < 2 || ctx.argc > 3) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    int st = atoi(ctx.argv[1]);

    // two arguments: motor number = 0
    if(ctx.argc == 2) {arg->rocket_data.fsm_state.update({(FSMState)st,0});}

    // three arguments: motor number given
    else if (ctx.argc == 3) {
        uint8_t motor = atoi(ctx.argv[2]);
        arg->rocket_data.fsm_state.update({(FSMState)st, motor});
    }
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
        Serial.print("New Serial Number: ");
        print_serial(arg->eeprom.data.serial);
        return MCommandExecutionResult::OK;
    }
    else if(!strcmp(ctx.argv[1], "get")){
        print_serial(arg->eeprom.data.serial);
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
        // ensure that given frequency is within the 70cm band 
        if (frequency > 450.0 || frequency < 420.0) { return MCommandExecutionResult::ERR_INVAL_ARG_RANGE; }
        
        // set eeprom frequency
        arg->eeprom.data.frequency = frequency;
        arg->eeprom.commit();

        // set telemetry frequency
        arg->tlm.setFrequency(frequency);

        Serial.print("New Frequency: ");
        Serial.print(arg->eeprom.data.frequency);
        Serial.println("MHz");
        return MCommandExecutionResult::OK;
    }
    else if(!strcmp(ctx.argv[1], "get")){
        Serial.print(arg->eeprom.data.frequency);
        Serial.println(" MHz");
        return MCommandExecutionResult::OK;
    }
    else{
        return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
    }
}

MCommandExecutionResult fsm_version(const MShellContext& ctx){
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;
    // expecting 2 (get) or 3 (set) arguments
    if(ctx.argc > 3) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    if(ctx.argc == 2){
        Serial.println(arg->fsm.get_cfg().version_num);
        return MCommandExecutionResult::OK;
    }
    else if(ctx.argc == 3){
        shell_cfg.version_num=atoi(ctx.argv[2]);
        Serial.print("Queued FSM Version: ");
        Serial.println(shell_cfg.version_num);
        return MCommandExecutionResult::OK;
    }
    return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
}

MCommandExecutionResult fsm_threshold(const MShellContext& ctx){
    if (ctx.argc>4 || ctx.argc<3) { return MCommandExecutionResult::ERR_INVAL_ARGC;}
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;

    // find correct threshold within threshold map
    int map_idx = -1;
    for (int i = 0; i<(sizeof(threshold_map)/sizeof(MapEntry)); i++){
        if (!strcmp(ctx.argv[2], threshold_map[i].name)){
            map_idx=i;
            break;
        }
    }
    if (map_idx==-1) { return MCommandExecutionResult::ERR_INVAL_ARGUMENT;}

    // get
    if (ctx.argc == 3){
        switch (threshold_map[map_idx].type){
            case DataType::BOOL:{
                bool val;
                memcpy(&val, (uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset, sizeof(bool));
                Serial.print(val);
                Serial.print(" ");
                const char * tf = val ? "(true)" : "(false)";
                Serial.println(tf);
                return MCommandExecutionResult::OK;
            }
            case DataType::UINT8:{
                uint8_t val;
                memcpy(&val, (uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset, sizeof(uint8_t));
                Serial.print(val);
                Serial.print(" ");
                Serial.println(threshold_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
            case DataType::FLOAT:{
                float val;
                memcpy(&val, (uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset, sizeof(float));
                Serial.print(val);
                Serial.print(" ");
                Serial.println(threshold_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
        }
    }
    // set
    if (ctx.argc == 4){
        switch (threshold_map[map_idx].type){
            case DataType::BOOL:{
                bool val = atoi(ctx.argv[3]) ? true : false;
                memcpy((uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset, &val, sizeof(bool));
                Serial.print("Queued: ");
                Serial.print(*(bool*)((uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset));
                Serial.print(" ");
                const char * tf = val ? "(true)" : "(false)";
                Serial.println(tf);
                return MCommandExecutionResult::OK;
            }
            case DataType::UINT8:{
                uint8_t val = atoi(ctx.argv[3]);
                memcpy((uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset, &val, sizeof(uint8_t));
                Serial.print("Queued: ");
                Serial.print(*(uint8_t*)((uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset));
                Serial.print(" ");
                Serial.println(threshold_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
            case DataType::FLOAT:{
                float val = atoff(ctx.argv[3]);
                memcpy((uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset, &val, sizeof(float));
                Serial.print("Queued: ");
                Serial.print(*(float*)((uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset));
                Serial.print(" ");
                Serial.println(threshold_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
        }
    }
    return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
}

MCommandExecutionResult fsm_channel(const MShellContext& ctx, uint8_t ch){
    if (ctx.argc>4 || ctx.argc<3) { return MCommandExecutionResult::ERR_INVAL_ARGC;}
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;

    // find correct parameter within channel map
    int map_idx = -1;
    for (int i = 0; i<(sizeof(channel_map)/sizeof(MapEntry)); i++){
        if (!strcmp(ctx.argv[2], channel_map[i].name)){
            map_idx=i;
            break;
        }
    }
    if (map_idx==-1) { return MCommandExecutionResult::ERR_INVAL_ARGUMENT;}

    // get
    if (ctx.argc == 3){
        switch (channel_map[map_idx].type){
            case DataType::BOOL:{
                bool val;
                memcpy(&val, (uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, sizeof(bool));
                Serial.print(val);
                Serial.print(" ");
                const char * tf = val ? "(true)" : "(false)";
                Serial.println(tf);
                return MCommandExecutionResult::OK;
            }
            case DataType::UINT8:{
                uint8_t val;
                memcpy(&val, (uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, sizeof(uint8_t));
                Serial.print(val);
                Serial.print(" ");
                Serial.println(threshold_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
            case DataType::FLOAT:{
                float val;
                memcpy(&val, (uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, sizeof(float));
                Serial.print(val);
                Serial.print(" ");
                Serial.println(threshold_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
            case DataType::FSMSTATE:{
                FSMState val;
                memcpy(&val, (uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, sizeof(FSMState));
                Serial.print(val);
                Serial.print(" (");
                Serial.print(state_names[val]);
                Serial.println(")");
                return MCommandExecutionResult::OK;
            }
        }
    }
    // set
    if (ctx.argc == 4){
        switch (channel_map[map_idx].type){
            case DataType::BOOL:{
                bool val = atoi(ctx.argv[3]) ? true : false;
                memcpy((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, &val, sizeof(bool));
                Serial.print("Queued: ");
                Serial.print(*(bool*)((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset));
                Serial.print(" ");
                const char * tf = val ? "(true)" : "(false)";
                Serial.println(tf);
                return MCommandExecutionResult::OK;
            }
            case DataType::UINT8:{
                uint8_t val = atoi(ctx.argv[3]);
                memcpy((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, &val, sizeof(uint8_t));
                Serial.print("Queued: ");
                Serial.print(*(uint8_t*)((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset));
                Serial.print(" ");
                Serial.println(threshold_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
            case DataType::FLOAT:{
                float val = atoff(ctx.argv[3]);
                memcpy((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, &val, sizeof(float));
                Serial.print("Queued: ");
                Serial.print(*(float*)((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset));
                Serial.print(" ");
                Serial.println(threshold_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
            case DataType::FSMSTATE:{
                FSMState val = (FSMState)atoi(ctx.argv[3]);

                // validate that given state is allowed for pyro events
                uint8_t valid = 0;
                for (int i = 0; i<(sizeof(allowed_trigger_states)/sizeof(FSMState)); i++){
                    if (val == allowed_trigger_states[i]) {valid = 1; break;}
                }
                if (valid == 0) {return MCommandExecutionResult::ERR_INVAL_ARGUMENT;}

                memcpy((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, &val, sizeof(FSMState));
                Serial.print("Queued: ");
                Serial.print(*(FSMState*)((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset));
                Serial.print(" (");
                Serial.print(state_names[val]);
                Serial.println(")");
                return MCommandExecutionResult::OK;
            }
        }
    }
    return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
}

MCommandExecutionResult fsm_help(){
    Serial.println("FSM Configuration: ");
    // insert more here

    return MCommandExecutionResult::OK;
}

MCommandExecutionResult fsm(const MShellContext& ctx){
    if (ctx.argc<2) {return MCommandExecutionResult::ERR_INVAL_ARGC;}
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;
    if (!strcmp(ctx.argv[1], "threshold")){
        return fsm_threshold(ctx);
    }
    else if (!strcmp(ctx.argv[1], "A")){
        return fsm_channel(ctx, 0);
    }
    else if (!strcmp(ctx.argv[1], "B")){
        return fsm_channel(ctx, 1);
    }
    else if (!strcmp(ctx.argv[1], "C")){
        return fsm_channel(ctx, 2);
    }
    else if (!strcmp(ctx.argv[1], "D")){
        return fsm_channel(ctx, 3);
    }
    else if (!strcmp(ctx.argv[1], "version")){
        return fsm_version(ctx);
    }
    else if (!strcmp(ctx.argv[1], "crc")){
        Serial.print("FSM CRC: ");
        Serial.println(arg->fsm.get_cfg().crc32);
        return MCommandExecutionResult::OK;
    }
    else if (!strcmp(ctx.argv[1], "commit")){
        if(arg->fsm.set_cfg(shell_cfg)) {return MCommandExecutionResult::OK;}
        return MCommandExecutionResult::ERR_INVAL_FSM;
    }
    else if (!strcmp(ctx.argv[1], "help")){
        return fsm_help();
    }
    return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
}

void m_shell_init_commands(MShell* sh) {
    sh->register_command("setfsm", set_fsm, "\tsetfsm <state:int> - Sets the FSM state to state <state>");
    sh->register_command("hi", hi_midas, "\t\thi <string> - Prints hi <string>");
    sh->register_command("serial", serial, "\tserial get - Get MIDAS serial number\n\t\tserial set <serialnumber:int> - Set MIDAS serial number");
    sh->register_command("frequency", frequency, "\tfrequency get - Get MIDAS telemetry frequency (in MHz)\n\t\tfrequency set <freq:float> - Set MIDAS telemetry frequency (in MHz)");
    sh->register_command("fsm", fsm, "\t\tThis command is used to configure the MIDAS FSM thresholds and pyro firing events. \n\t\tUse through the official MIDAS-Base software package is recommended, or experienced users may enter \"fsm help\" for more information.");
    // add calibration commands
}