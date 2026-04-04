#include <systems.h>
#include <string>


FSMConfiguration shell_cfg;

enum class DataType {
    FLOAT, DOUBLE, BOOL, UINT8, FSMSTATE
};

struct MapEntry {
    const char* name;
    size_t offset;
    DataType type;
};

static constexpr MapEntry threshold_map[] = {
    {"PYRO_FIRE_T", offsetof(FSMUserThresholds, pyro_fire_t), DataType::FLOAT},
    {"MAIN_ALT", offsetof(FSMUserThresholds, main_alt), DataType::FLOAT},
    {"CRUISE_LOCKOUT_EN", offsetof(FSMUserThresholds, cruise_lockout_en), DataType::BOOL},
};

static constexpr MapEntry channel_map[] = {
    {"ENABLE", offsetof(FSMPyroAction, enable), DataType::BOOL},
    {"FSM_TRIGGER", offsetof(FSMPyroAction, fsm_trigger), DataType::FSMSTATE},
    {"DELAY", offsetof(FSMPyroAction, delay), DataType::DOUBLE},
    {"MAX_TILT", offsetof(FSMPyroAction, max_tilt), DataType::FLOAT},
    {"AFTER_MOTOR", offsetof(FSMPyroAction, after_motor), DataType::UINT8},
    {"LAUNCH_T_GT", offsetof(FSMPyroAction, launch_t_gt), DataType::FLOAT},
    {"LAUNCH_T_LT", offsetof(FSMPyroAction, launch_t_lt), DataType::FLOAT},
    {"VX_MIN", offsetof(FSMPyroAction, vx_min), DataType::FLOAT},
    {"VX_MAX", offsetof(FSMPyroAction, vx_max), DataType::FLOAT},
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
        Serial.print("Serial Number: ");
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
        Serial.print("Frequency: ");
        Serial.print(arg->eeprom.data.frequency);
        Serial.println("MHz");
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
        Serial.print("FSM Version: ");
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

MCommandExecutionResult fsm_calculate(const MShellContext& ctx){
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;
    // Expecting just "fsm calculate"
    if(ctx.argc > 2) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    uint32_t my_fsm_crc = FSMConfiguration::calculate_crc(shell_cfg);
    Serial.println(my_fsm_crc);
    return MCommandExecutionResult::OK;
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
                memcpy(&val, (uint8_t*)(&arg->fsm.get_cfg().thresholds)+threshold_map[map_idx].offset, sizeof(bool));
                Serial.println(val);
                return MCommandExecutionResult::OK;
            }
            case DataType::UINT8:{
                uint8_t val;
                memcpy(&val, (uint8_t*)(&arg->fsm.get_cfg().thresholds)+threshold_map[map_idx].offset, sizeof(uint8_t));
                Serial.println(val);
                return MCommandExecutionResult::OK;
            }
            case DataType::FLOAT:{
                float val;
                memcpy(&val, (uint8_t*)(&arg->fsm.get_cfg().thresholds)+threshold_map[map_idx].offset, sizeof(float));
                Serial.println(val);
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
                Serial.println(*(bool*)((uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset));
                return MCommandExecutionResult::OK;
            }
            case DataType::UINT8:{
                uint8_t val = atoi(ctx.argv[3]);
                memcpy((uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset, &val, sizeof(uint8_t));
                Serial.print("Queued: ");
                Serial.println(*(uint8_t*)((uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset));
                return MCommandExecutionResult::OK;
            }
            case DataType::FLOAT:{
                float val = atoff(ctx.argv[3]);
                memcpy((uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset, &val, sizeof(float));
                Serial.print("Queued: ");
                Serial.println(*(float*)((uint8_t*)(&shell_cfg.thresholds)+threshold_map[map_idx].offset));
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
                memcpy(&val, (uint8_t*)(&arg->fsm.get_cfg().pyro_actions[ch])+channel_map[map_idx].offset, sizeof(bool));
                Serial.println(val);
                return MCommandExecutionResult::OK;
            }
            case DataType::UINT8:{
                uint8_t val;
                memcpy(&val, (uint8_t*)(&arg->fsm.get_cfg().pyro_actions[ch])+channel_map[map_idx].offset, sizeof(uint8_t));
                Serial.println(val);
                return MCommandExecutionResult::OK;
            }
            case DataType::FLOAT:{
                float val;
                memcpy(&val, (uint8_t*)(&arg->fsm.get_cfg().pyro_actions[ch])+channel_map[map_idx].offset, sizeof(float));
                Serial.println(val);
                return MCommandExecutionResult::OK;
            }
            case DataType::DOUBLE:{
                double val;
                memcpy(&val, (uint8_t*)(&arg->fsm.get_cfg().pyro_actions[ch])+channel_map[map_idx].offset, sizeof(double));
                Serial.println(val);
                return MCommandExecutionResult::OK;
            }
            case DataType::FSMSTATE:{
                FSMState val;
                memcpy(&val, (uint8_t*)(&arg->fsm.get_cfg().pyro_actions[ch])+channel_map[map_idx].offset, sizeof(FSMState));
                Serial.println(val);
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
                Serial.println(*(bool*)((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset));
                return MCommandExecutionResult::OK;
            }
            case DataType::UINT8:{
                uint8_t val = atoi(ctx.argv[3]);
                memcpy((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, &val, sizeof(uint8_t));
                Serial.print("Queued: ");
                Serial.println(*(uint8_t*)((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset));
                return MCommandExecutionResult::OK;
            }
            case DataType::FLOAT:{
                float val = atoff(ctx.argv[3]);
                memcpy((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, &val, sizeof(float));
                Serial.print("Queued: ");
                Serial.println(*(float*)((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset));
                return MCommandExecutionResult::OK;
            }
            case DataType::DOUBLE:{
                double val = atof(ctx.argv[3]);
                memcpy((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, &val, sizeof(double));
                Serial.print("Queued: ");
                Serial.println(*(double*)((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset));
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
                Serial.println(*(FSMState*)((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset));
                return MCommandExecutionResult::OK;
            }
        }
    }
    return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
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
    else if (!strcmp(ctx.argv[1], "calculate")) {
        return fsm_calculate(ctx);
    }
    else if (!strcmp(ctx.argv[1], "crc")){
        Serial.print("FSM CRC: ");
        Serial.println(arg->fsm.get_cfg().crc32);
        return MCommandExecutionResult::OK;
    }
    else if (!strcmp(ctx.argv[1], "commit")){
        // Set the config's crc
        uint32_t crc = std::stoul(ctx.argv[2]);
        shell_cfg.crc32 = crc;
        if(arg->fsm.set_cfg(shell_cfg)) {
            arg->led.set(LED::RED, LOW);
            arg->rocket_data.err_flags.fsm_crc_err = false; // Clear CRC error flag

            // Commit to EEPROM
            arg->eeprom.data.fsm_config = arg->fsm.get_cfg();
            arg->eeprom.commit();

            return MCommandExecutionResult::OK;
        }
        return MCommandExecutionResult::ERR_INVAL_FSM;
    }
    return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
}

void m_shell_load_fsm_config(const FSMConfiguration& fsm_cfg) {
    shell_cfg = fsm_cfg;
}

void m_shell_init_commands(MShell* sh) {
    // sh->register_command("setfsm", set_fsm, "\tsetfsm <state:int> - Sets the FSM state to state <state>");
    sh->register_command("hi", hi_midas, "\t\thi <string> - Prints hi <string>");
    sh->register_command("serial", serial, "\tserial get - Get MIDAS serial number\n\t\tserial set <serialnumber:int> - Set MIDAS serial number");
    sh->register_command("frequency", frequency, "\tfrequency get - Get MIDAS telemetry frequency (in MHz)\n\t\tfrequency set <freq:float> - Set MIDAS telemetry frequency (in MHz)");
    sh->register_command("fsm", fsm, "\t\tThis command is only supported when used with the official MIDAS-Base software package.");
    // add calibration commands
}