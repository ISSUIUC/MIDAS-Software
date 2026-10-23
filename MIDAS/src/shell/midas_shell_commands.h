#include <flight-systems/systems.h>
#include <string>
#include <SD_MMC.h>
#include <logging/log_format.h>
#include "logging/log_checksum.h"
#include "logging/esp_eeprom_checksum.h"

/**
 * @brief Global shell configuration structure holding the staged/queued 
 * Finite State Machine (FSM) configuration parameters.
 */
FSMConfiguration shell_cfg;

/**
 * @brief Supported data types for flight software parameters mapped in the command shell.
 */
enum class DataType {
    FLOAT,      /**< 32-bit floating point value */
    DOUBLE,     /**< 64-bit floating point value */
    BOOL,       /**< Boolean flag (true/false) */
    UINT8,      /**< 8-bit unsigned integer */
    FSMSTATE    /**< Enumerated Finite State Machine state */
};

/**
 * @brief Entry format for mapping string identifiers to internal structure fields via memory offsets.
 */
struct MapEntry {
    const char* name;   /**< String command identifier passed via the shell */
    size_t offset;      /**< Memory offset of the field within its parent structure */
    DataType type;      /**< Underlying data type of the target field */
    const char* unit;   /**< Human-readable unit symbol (e.g., "ms", "m", "degrees") */
};

/**
 * @brief Lookup map linking global FSM system thresholds to their memory offsets and data types.
 */
static constexpr MapEntry threshold_map[] = {
    {"PYRO_FIRE_T", offsetof(FSMUserThresholds, pyro_fire_t), DataType::FLOAT, "ms"},
    {"MAIN_ALT", offsetof(FSMUserThresholds, main_alt), DataType::FLOAT, "m"},
    {"CRUISE_LOCKOUT_EN", offsetof(FSMUserThresholds, cruise_lockout_en), DataType::BOOL, ""},
};

/**
 * @brief Lookup map linking pyrotechnic channel configuration fields to their memory offsets and data types.
 */
static constexpr MapEntry channel_map[] = {
    {"ENABLE", offsetof(FSMPyroAction, enable), DataType::BOOL, ""},
    {"FSM_TRIGGER", offsetof(FSMPyroAction, fsm_trigger), DataType::FSMSTATE, ""},
    {"DELAY", offsetof(FSMPyroAction, delay), DataType::DOUBLE, "ms"},
    {"MAX_TILT", offsetof(FSMPyroAction, max_tilt), DataType::FLOAT, "degrees"},
    {"AFTER_MOTOR", offsetof(FSMPyroAction, after_motor), DataType::UINT8, ""},
    {"LAUNCH_T_GT", offsetof(FSMPyroAction, launch_t_gt), DataType::FLOAT, "ms"},
    {"LAUNCH_T_LT", offsetof(FSMPyroAction, launch_t_lt), DataType::FLOAT, "ms"},
    {"VX_MIN", offsetof(FSMPyroAction, vx_min), DataType::FLOAT, "m/s"},
    {"VX_MAX", offsetof(FSMPyroAction, vx_max), DataType::FLOAT, "m/s"},
};

/**
 * @brief String representations of the flight FSM states for telemetry reporting and shell outputs.
 */
static constexpr const char * state_names[] = {
    "SAFE",
    "PYRO_TEST",
    "ARMED",
    "BOOST",
    "COAST",
    "DROGUE",
    "MAIN",
    "LANDED",
};

/**
 * @brief Valid flight states authorized to safely trigger pyrotechnic deployment events.
 */
static constexpr FSMState allowed_trigger_states[] = {
    FSMState::STATE_COAST, FSMState::STATE_DROGUE, FSMState::STATE_MAIN
};

/**
 * @brief Helper utility to print an 8-bit board serial identifier with zero-padded 3-digit formatting over Serial.
 * @param serial The raw 8-bit unsigned integer serial number to format and output.
 */
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

/**
 * @brief Manual override command to force alter the active flight state machine execution profile.
 * @param ctx The current shell context parsed containing arguments.
 * argv[1]: target state integer ID
 * argv[2] (Optional): specific target motor index
 * @return MCommandExecutionResult Execution status code.
 */
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

/**
 * @brief Simple handshake shell command to echo an input string for debugging communication lines.
 * @param ctx Shell context containing potential arguments to echo back.
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult hi_midas(const MShellContext& ctx) {
    Serial.print("hi ");
    // if no second argument, return
    if(ctx.argc != 2) { return MCommandExecutionResult::OK; }
    Serial.println(ctx.argv[1]);
    return MCommandExecutionResult::OK;
}

/**
 * @brief Shell command interface to retrieve or rewrite the system serial identifier in non-volatile storage.
 * @param ctx Shell context. Usage: `serial get` or `serial set <value>`
 * @return MCommandExecutionResult Execution status code.
 */
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

/**
 * @brief Shell command interface to fetch or modify the active 70cm radio telemetry band center frequency.
 * @details Limits operational input to local safety brackets (420.0 MHz - 450.0 MHz) prior to updating hardware.
 * @param ctx Shell context. Usage: `frequency get` or `frequency set <value>`
 * @return MCommandExecutionResult Execution status code.
 */
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

/**
 * @brief Shell command to examine active configuration variants or stage a new software version baseline.
 * @param ctx Shell context. Usage: `fsm version` or `fsm version <new_version>`
 * @return MCommandExecutionResult Execution status code.
 */
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

/**
 * @brief Computes a validation checksum validation hash over the queued temporary shell configuration buffer.
 * @param ctx Shell context. Usage: `fsm calculate`
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult fsm_calculate(const MShellContext& ctx){
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;
    // Expecting just "fsm calculate"
    if(ctx.argc > 2) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    uint32_t my_fsm_crc = FSMConfiguration::calculate_crc(shell_cfg);
    Serial.println(my_fsm_crc);
    return MCommandExecutionResult::OK;
}

/**
 * @brief Handles reading and writing individual parameters located inside the global flight safety limits block.
 * @param ctx Shell context containing arguments for reading/writing global bounds structures.
 * @return MCommandExecutionResult Execution status code.
 */
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

/**
 * @brief Modifies or queries operational profiles assigned to individual physical output pyro switches.
 * @param ctx Shell context containing programmatic configuration elements.
 * @param ch  The targeted hardware pyro index identifier (0=A, 1=B, 2=C, 3=D).
 * @return MCommandExecutionResult Execution status code.
 */
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
                Serial.println(channel_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
            case DataType::FLOAT:{
                float val;
                memcpy(&val, (uint8_t*)(&arg->fsm.get_cfg().pyro_actions[ch])+channel_map[map_idx].offset, sizeof(float));
                Serial.print(val);
                Serial.print(" ");
                Serial.println(channel_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
            case DataType::DOUBLE:{
                double val;
                memcpy(&val, (uint8_t*)(&arg->fsm.get_cfg().pyro_actions[ch])+channel_map[map_idx].offset, sizeof(double));
                Serial.print(val);
                Serial.print(" ");
                Serial.println(channel_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
            case DataType::FSMSTATE:{
                FSMState val;
                memcpy(&val, (uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, sizeof(FSMState));
                Serial.print(val);
                if (val >= FSMState::FSM_STATE_COUNT || val < 0) {
                    Serial.println(""); 
                    return MCommandExecutionResult::ERR_INVAL_EEPROM;
                }
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
                Serial.println(channel_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
            case DataType::FLOAT:{
                float val = atoff(ctx.argv[3]);
                memcpy((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, &val, sizeof(float));
                Serial.print("Queued: ");
                Serial.print(*(float*)((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset));
                Serial.print(" ");
                Serial.println(channel_map[map_idx].unit);
                return MCommandExecutionResult::OK;
            }
            case DataType::DOUBLE:{
                double val = atof(ctx.argv[3]);
                memcpy((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset, &val, sizeof(double));
                Serial.print("Queued: ");
                Serial.print(*(double*)((uint8_t*)(&shell_cfg.pyro_actions[ch])+channel_map[map_idx].offset));
                Serial.print(" ");
                Serial.println(channel_map[map_idx].unit);
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

/**
 * @brief Validates, flushes, and commits the staged configuration data into non-volatile EEPROM storage.
 * @details Resets internal FSM health condition structures and status indicator lights on successful verification.
 * @param ctx Shell context containing target integrity CRC hashes for security validation.
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult fsm_commit(const MShellContext& ctx){
    if (ctx.argc != 3) {return MCommandExecutionResult::ERR_INVAL_ARGC;}
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;

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

/**
 * @brief Outputs structural descriptions and syntax examples for all FSM terminal commands over Serial.
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult fsm_help(){
    Serial.println("FSM Configuration Help:\n");

    Serial.println("General FSM Commands:");
    Serial.println("\tfsm commit [uint32_t crc] - save queued FSM config to EEPROM");
    Serial.println("\tfsm calculate - calculate crc for queued FSM config");
    Serial.println("\tfsm crc - display crc for current FSM config");
    Serial.println("\tfsm version [OPTIONAL uint8_t version] - display current FSM version or queue new version");
    
    Serial.println("FSM Threshold Configuration:");
    Serial.println("\tfsm threshold CRUISE_LOCKOUT_EN [OPTIONAL bool] - get current cruise lockout status or queue new value");
    Serial.println("\tfsm threshold MAIN_ALT [OPTIONAL float] - get current main deploy altitude (m) or queue new value");
    Serial.println("\tfsm threshold PYRO_FIRE_T [OPTIONAL float] - get current pyro firing time (ms) or set new value");

    Serial.println("FSM Channel Configuration:");
    Serial.println("\tfsm [A/B/C/D] ENABLE [OPTIONAL bool] - get current channel enable status or queue new value");
    Serial.println("\tfsm [A/B/C/D] FSM_TRIGGER [OPTIONAL int] - get current channel trigger state or queue new value");
    Serial.println("\tfsm [A/B/C/D] DELAY [OPTIONAL double] - get current channel firing delay or queue new value");
    Serial.println("\tfsm [A/B/C/D] MAX_TILT [OPTIONAL float] - get current channel maximum tilt requirement or queue new value");
    Serial.println("\tfsm [A/B/C/D] AFTER_MOTOR [OPTIONAL int] - get current channel motor count requirement or queue new value");
    Serial.println("\tfsm [A/B/C/D] LAUNCH_T_GT [OPTIONAL float] - get current channel minimum launch time requirement or queue new value");
    Serial.println("\tfsm [A/B/C/D] LAUNCH_T_LT [OPTIONAL float] - get current channel maximum launch time requirement or queue new value");
    Serial.println("\tfsm [A/B/C/D] VX_MIN [OPTIONAL float] - get current channel minimum velocity requirement or queue new value");
    Serial.println("\tfsm [A/B/C/D] VX_MAX [OPTIONAL float] - get current channel maximum velocity requirement or queue new value");

    return MCommandExecutionResult::OK;
}

/**
 * @brief Router terminal handle processing sub-demands targeted toward flight profiling rules.
 * @param ctx Shell context containing arguments.
 * @return MCommandExecutionResult Execution status code.
 */
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
        Serial.println(arg->fsm.get_cfg().crc32);
        return MCommandExecutionResult::OK;
    }
    else if (!strcmp(ctx.argv[1], "commit")){
        return fsm_commit(ctx);
    }
    else if (!strcmp(ctx.argv[1], "help")){
        return fsm_help();
    }
    return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
}

/**
 * @brief Initializes the localized working shell buffer configuration image from runtime configurations.
 * @param fsm_cfg Reference source configuration containing initialization state values.
 */
void m_shell_load_fsm_config(const FSMConfiguration& fsm_cfg) {
    shell_cfg = fsm_cfg;
}

/**
 * @brief Shell command to query and structure file catalog details currently visible within storage card partitions.
 * @param ctx Shell context. Usage: `ls`
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult cmd_ls(const MShellContext& ctx) {
    if(ctx.argc != 1) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    File root = SD_MMC.open("/");

    if(!root) {
        Serial.println("Failed to open /");
        return MCommandExecutionResult::ERR_FS_FAIL_OPEN;
    }
    
    uint64_t used_size = SD_MMC.usedBytes();
    uint64_t card_size = SD_MMC.totalBytes();

    Serial.print("Memory Usage: ");
    Serial.print(used_size);
    Serial.print("B / ");
    Serial.print(card_size);
    Serial.println("B");

    Serial.println(" | name (size) ");
    Serial.println("-+-------------");
    File file = root.openNextFile();
    while (file) {
        Serial.print(" | ");
        Serial.print(file.name());
        Serial.print(" (");
        Serial.print(file.size());
        Serial.println(" bytes)");
        file.close();
        file = root.openNextFile();
    }

    return MCommandExecutionResult::OK;
}

/**
 * @brief Streaming reader outputting full raw contents of an explicit file asset target directly to serial.
 * @param ctx Shell context containing targeted local string file paths. Usage: `read <filename>`
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult cmd_read(const MShellContext& ctx) {
    if (ctx.argc != 2) { return MCommandExecutionResult::ERR_INVAL_ARGC; }

    File file = SD_MMC.open(ctx.argv[1], FILE_READ);

    if(!file) {
        Serial.print("Failed to open ");
        Serial.println(ctx.argv[1]);
        return MCommandExecutionResult::ERR_FS_FAIL_OPEN;
    }

    while(file.available()) {
        Serial.write(file.read());
    }
    file.close();
    return MCommandExecutionResult::OK;
}

/**
 * @brief Helper evaluation block verifying if a given directory file targets the live logging output metrics stream.
 * @param arg Active flight vehicle operational class instances.
 * @param path Extracted folder file lookup strings under comparative processing checks.
 * @return true If the file matches active logs; false otherwise.
 */
static bool is_active_log(RocketSystems* arg, const char* path) {
    const char* basename = strrchr(path, '/');
    basename = basename ? basename + 1 : path;

    char bin[20];
    char meta[20];
    snprintf(bin,  sizeof(bin),  "data%u.bin",  arg->log_sink.current_file_no - 1);
    snprintf(meta, sizeof(meta), "data%u.meta", arg->log_sink.current_file_no - 1);
    return !strcmp(basename, bin) || !strcmp(basename, meta);
}

/**
 * @brief Safety sweep utility targeting deletion tasks spanning passive log file fragments across media partitions.
 * @details Retains and isolates the currently running data structure files to guard active recording sessions.
 * @param arg Hardware telemetry core coordination state machine structures.
 * @return true On clean execution sweeps; false if hardware blockades disrupt operations.
 */
bool delete_all_flash(RocketSystems* arg) {
    File root = SD_MMC.open("/");
    while (true) {
        File file = root.openNextFile();
        if (!file) {
            break; // No more files
        }
        String fullpath = "/" + String(file.name());
        file.close();
        file.flush();
        if (is_active_log(arg, fullpath.c_str())) {
            Serial.print("Skipping active log: ");
            Serial.println(fullpath);
            continue;
        }
        Serial.print("Deleting file: ");
        Serial.println(fullpath);
        if(!SD_MMC.remove(fullpath.c_str())) {
            Serial.print("Failed to delete ");
            Serial.println(fullpath);
            return false;
        }
    }
    return true;
}

/**
 * @brief Packages and streams unified high-fidelity launch session artifacts down telemetry pipelines.
 * @details Interleaves structure elements, system metadata tracking logs, and direct checksum validation.
 * @param ctx Shell context container parameters. Usage: `lfd <data_prefix>` (e.g., `lfd data17`)
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult cmd_lfd(const MShellContext& ctx) {
    // Launch File Dump: Outputs a .launch file as binary data over serial

    // The new .launch format is as follows:
    // LAUNCH <version_no>
    // FILE <original_filename>
    // CHECKSUM <log_checksum> <eeprom_checksum>
    // META <meta file size>
    // BIN <bin file size>
    // <meta file content>
    // <bin file content>

    if (ctx.argc != 2 && ctx.argc != 3) {  }
    bool stop_after_header_print = false;
    if(ctx.argc == 3) {
        if(!strcmp(ctx.argv[2], "--head")) {
            stop_after_header_print = true;
        } else {
            return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
        }
    } else if (ctx.argc != 2) {
        return MCommandExecutionResult::ERR_INVAL_ARGC;
    }


    String bin_path = "/" + String(ctx.argv[1]) + ".bin";
    String meta_path = "/" + String(ctx.argv[1]) + ".meta";
    File bin_file = SD_MMC.open(bin_path.c_str(), FILE_READ);
    File meta_file = SD_MMC.open(meta_path.c_str(), FILE_READ);

    if(!bin_file) {
        Serial.println("Failed to open .bin file!");
        return MCommandExecutionResult::ERR_FS_FAIL_OPEN;
    }

    if(!meta_file) {
        Serial.println("Failed to open .meta file!");
        return MCommandExecutionResult::ERR_FS_FAIL_OPEN;
    }

    size_t size_ptr = LOG_FMT_VERSION; // We write multiple uint32_ts, so for serial.write to print it properly we need to store it
    
    // Start writing header
    Serial.write("LAUNCH "); Serial.write((uint8_t*)&size_ptr, sizeof(size_t)); Serial.write('\n');

    // File name
    Serial.write("FILE "); Serial.write(ctx.argv[1]); Serial.write('\n');
    
    // Log checksum
    size_ptr = LOG_CHECKSUM;
    Serial.write("CHECKSUM "); Serial.write((uint8_t*)&size_ptr, sizeof(size_t)); Serial.write(" "); 
    
    // EEPROM checksum
    size_ptr = EEPROM_CHECKSUM;
    Serial.write((uint8_t*)&size_ptr, sizeof(size_t)); Serial.write('\n');

    // Meta file size
    size_ptr = meta_file.size();
    Serial.write("META "); Serial.write((uint8_t*)&size_ptr, sizeof(size_t)); Serial.write('\n');

    // Binary file size
    size_ptr = bin_file.size();
    Serial.write("BIN "); Serial.write((uint8_t*)&size_ptr, sizeof(size_t)); Serial.write('\n');

    Serial.flush();

    if(stop_after_header_print) { return MCommandExecutionResult::OK; }

    // Then dump meta and bin files
    while(meta_file.available()) {
        Serial.write(meta_file.read());
    }
    meta_file.close();

    while(bin_file.available()) {
        Serial.write(bin_file.read());
    }
    bin_file.close();

    Serial.flush();

    return MCommandExecutionResult::OK;
}

/**
 * @brief Responds with the flight computer platform identifier signature over active standard serial out.
 * @param ctx Shell context. Usage: `ident`
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult ident_midas(const MShellContext& ctx) {
    Serial.println("IDENT_RESPONSE:MIDAS_MINI");
    return MCommandExecutionResult::OK;
}

/**
 * @brief Shell command executing removal routines handling isolated singular files or widespread safety flash wipes.
 * @param ctx Shell context. Usage: `rm <filename>` or `rm *`
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult cmd_rm(const MShellContext& ctx) {
    if (ctx.argc != 2) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;

    // Special case: doing `rm *` removes all files:
    if (!strcmp(ctx.argv[1], "*")) {
        if(!delete_all_flash(arg)) {
            return MCommandExecutionResult::ERR_FS_FAIL_OPEN;
        }
        return MCommandExecutionResult::OK;
    }

    if (is_active_log(arg, ctx.argv[1])) {
        Serial.print("File is currently being written to (forbidden): ");
        Serial.println(ctx.argv[1]);
        return MCommandExecutionResult::ERR_FORBIDDEN;
    }

    if(!SD_MMC.remove(ctx.argv[1])) {
        Serial.print("Failed to delete ");
        Serial.println(ctx.argv[1]);
        return MCommandExecutionResult::ERR_FS_FAIL_OPEN;
    }
    return MCommandExecutionResult::OK;
}

/**
 * @brief Enters an automated runtime state loop to update sensor internal zero points.
 * @param ctx Shell context designating specific hardware channels. Usage: `calibrate [xl/mag]`
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult m_calibration(const MShellContext& ctx) {
    if (ctx.argc != 2) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;

    if (!strcmp(ctx.argv[1], "xl") || !strcmp(ctx.argv[1], "accel") || !strcmp(ctx.argv[1], "accelerometer")){
        arg->sensors.imu.begin_calibration(arg->buzzer);
        return MCommandExecutionResult::OK;
    }
    else if (!strcmp(ctx.argv[1], "mag") || !strcmp(ctx.argv[1], "magnetometer")){
        arg->sensors.magnetometer.begin_calibration(arg->buzzer);
        return MCommandExecutionResult::OK;
    }
    
    return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
}

/**
 * @brief Prints documentation tracking direct programmatic hardware component correction offsets.
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult calibset_help(){
    Serial.println("CalibSet Help:\n");

    Serial.println("\tcalibset help - shows this menu\n");
    
    Serial.println("Accelerometer (LSM6DSV320X)");
    Serial.println("\tcalibset accelerometer [x/y/z] [float] - Sets the LSM6DSV320X's high g accelerometer offset along an axis (x/y/z)");
    Serial.println("\tcalibset accel [x/y/z] [float] - Macro for 'calibset accelerometer'");
    Serial.println("\tcalibset xl [x/y/z] [float] - Macro for 'calibset accelerometer'");
    
    Serial.println("Magnetometer (MMC5983MA)");
    Serial.println("\tcalibset magnetometer si.[x/y/z] [float] - Sets the MMC5983MA's 'soft iron' offset along an axis (x/y/z)");
    Serial.println("\tcalibset magnetometer hi.[x/y/z] [float] - Sets the MMC5983MA's 'hard iron' offset along an axis (x/y/z)");
    Serial.println("\tcalibset mag [si/hi].[x/y/z] [float] - Macro for 'calibset magnetometer'");

    return MCommandExecutionResult::OK;
}

/**
 * @brief Command handler interface allowing manual software injection of calculated sensor bias matrices.
 * @details Commits modified hard-iron, soft-iron, or high-G baseline bias vectors safely into EEPROM memory structures.
 * @param ctx Shell context containing programmatic configuration elements.
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult m_calibset(const MShellContext& ctx) {

    if (ctx.argc < 2) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    if (!strcmp(ctx.argv[1], "help")){
        return calibset_help();
    }

    if (ctx.argc != 4) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;

    if (!strcmp(ctx.argv[1], "xl") || !strcmp(ctx.argv[1], "accel") || !strcmp(ctx.argv[1], "accelerometer")){
        Acceleration cur_bias =  arg->sensors.imu.calibration_sensor_bias;
        float val = atoff(ctx.argv[3]);
        if(!strcmp(ctx.argv[2], "x")) { cur_bias.ax = val; }
        else if(!strcmp(ctx.argv[2], "y")) { cur_bias.ay = val; }
        else if(!strcmp(ctx.argv[2], "z")) { cur_bias.az = val; }
        else { return MCommandExecutionResult::ERR_INVAL_ARGUMENT; }
        arg->sensors.imu.calibration_sensor_bias = cur_bias;
        arg->eeprom.data.lsm6dsv320x_hg_xl_bias = cur_bias;
        arg->eeprom.commit();
        return MCommandExecutionResult::OK;
    }
    else if (!strcmp(ctx.argv[1], "mag") || !strcmp(ctx.argv[1], "magnetometer")){
        Magnetometer cur_hi =  arg->sensors.magnetometer.calibration_bias_hardiron;
        Magnetometer cur_si =  arg->sensors.magnetometer.calibration_bias_softiron;
        float val = atoff(ctx.argv[3]);
        if(!strcmp(ctx.argv[2], "si.x")) { cur_si.mx = val; }
        else if(!strcmp(ctx.argv[2], "si.y")) { cur_si.my = val; }
        else if(!strcmp(ctx.argv[2], "si.z")) { cur_si.mz = val; }
        else if(!strcmp(ctx.argv[2], "hi.x")) { cur_hi.mx = val; }
        else if(!strcmp(ctx.argv[2], "hi.y")) { cur_hi.my = val; }
        else if(!strcmp(ctx.argv[2], "hi.z")) { cur_hi.mz = val; }
        else { return MCommandExecutionResult::ERR_INVAL_ARGUMENT; }
        arg->eeprom.data.mmc5983ma_softiron_bias = cur_si;
        arg->eeprom.data.mmc5983ma_softiron_bias = cur_hi;
        arg->sensors.magnetometer.calibration_bias_softiron = cur_si;
        arg->sensors.magnetometer.calibration_bias_hardiron = cur_hi;
        arg->eeprom.commit();
        return MCommandExecutionResult::OK;
    }
    
    return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
}

/**
 * @brief Shell interface wrapper to fetch or adjust the rolling log prefix counter tracker.
 * @details Prevents log name collisions during power cycle resets by tracking historical file numbers.
 * @param ctx Shell context. Usage: `sdfn` (get) or `sdfn <file_num>` (set)
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult m_sdfn(const MShellContext& ctx) {
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;
    if(ctx.argc == 1) {
        Serial.println(arg->eeprom.data.sd_file_num_last);
        return MCommandExecutionResult::OK;
    }

    if(ctx.argc == 2) {
        int val = atoi(ctx.argv[1]);
        arg->eeprom.data.sd_file_num_last = val;
        arg->eeprom.commit();
        return MCommandExecutionResult::OK;
    }
    return MCommandExecutionResult::ERR_INVAL_ARGC;
}

/**
 * @brief Main execution command registry mapping terminal text command tokens to software execution handles.
 * @param sh Target shell processing structure passing operational registers.
 */
void m_shell_init_commands(MShell* sh) {
    // sh->register_command("setfsm", set_fsm, "\tsetfsm <state:int> - Sets the FSM state to state <state>");
    sh->register_command("hi", hi_midas, "\t\thi <string> - Prints hi <string>");
    sh->register_command("serial", serial, "\tserial get - Get MIDAS serial number\n\t\tserial set <serialnumber:int> - Set MIDAS serial number");
    sh->register_command("frequency", frequency, "\tfrequency get - Get MIDAS telemetry frequency (in MHz)\n\t\tfrequency set <freq:float> - Set MIDAS telemetry frequency (in MHz)");
    sh->register_command("fsm", fsm, "\t\tThis command is used to configure the MIDAS FSM thresholds and pyro firing events. \n\t\tUse through the official MIDAS-Base software package is recommended, or experienced users may enter \"fsm help\" for more information.");
    sh->register_command("sdfn", m_sdfn, "\tsdfn - SD File Number, used to get or set the 'last SD fileno' value, used for seeking the next data file name");

    // File commands
    sh->register_command("ls", cmd_ls, "\tls - List all files in the mounted LogSink");
    sh->register_command("read", cmd_read, "\tread <file> - Reads a file from the mounted LogSink");
    sh->register_command("lfd", cmd_lfd, "\tlfd <dataf> - Given a data file string (i.e. 'data17'), does a 'launch file dump', outputting a .launch file");
    sh->register_command("rm", cmd_rm, "\trm <file> - Deletes a file from the mounted LogSink");

    // sensor calibration
    sh->register_command("calibrate", m_calibration, "\tcalibrate <sensor> - Inits calibration for a sensor, accepts sensor shorthand, i.e. 'xl' or 'mag'");
    sh->register_command("calibset", m_calibset, "\tcalibset <sensor> <axis> <value> - Sets a calibration value for a sensor's axis.\n\tcalibset help - Display calibset's possible options");
    
    //Identify
    sh->register_command("ident", ident_midas, "\tIdentify MIDAS");
}