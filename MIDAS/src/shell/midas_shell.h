#pragma once
#include <cstdint>
#include <array>
#include <Arduino.h>
#include <string.h>

/**
 * @brief Return codes indicating the final execution status of an interactive shell command.
 */
enum class MCommandExecutionResult {
    // Generic
    OK = 0,                  /**< The command executed successfully without errors */
    ERR_UNSPECIFIED = 1,     /**< Execution failed due to an unhandled or general system error */
    ERR_INVALID_CMD = 2,     /**< The command string format or structure could not be parsed */
    ERR_NO_CMD = 3,          /**< The parsed command keyword does not match any registered handler */
    ERR_INVAL_ARGC = 4,      /**< The number of arguments provided does not match the command signature */
    ERR_INVAL_ARGUMENT = 5,  /**< One or more provided arguments failed generic validation checks */
    ERR_INVAL_ARG_RANGE = 6, /**< One or more numeric parameters fall outside acceptable functional limits */
    ERR_INVAL_FSM = 7,       /**< State transition profile commitment validation failed */
    ERR_INVAL_EEPROM = 8,    /**< Non-volatile memory storage data is corrupted or contains garbage values */
    ERR_FS_FAIL_OPEN = 9,    /**< File or directory target could not be loaded inside the local filesystem */
    ERR_FORBIDDEN = 10       /**< Requested operational sequence is locked or blocked due to current flight status */
};

/**
 * @brief Parameter envelope containing argument references and external system contexts passed into command functions.
 */
struct MShellContext {
    uint8_t argc;                 /**< Total tokenized argument count including the command keyword identifier itself */
    const char** argv;            /**< Array of null-terminated C-string parameter strings parsed from the command line */
    const void* sysarg = nullptr; /**< Runtime core system state reference object pointer (Opaque handle to RocketSystems*) */
};

/**
 * @brief Function pointer signature definitions corresponding to valid command executor callbacks.
 */
using MShellExecutor = MCommandExecutionResult(*)(const MShellContext&);

/**
 * @brief Storage structure linking an input command invocation keyword to its designated callback logic and usage menus.
 */
struct MShellCommand {
    const char* cmd;        /**< Unique string token keyword identifying the active command trigger */
    MShellExecutor fn;      /**< Target function pointer executed upon a successful identifier keyword match */
    const char* help_text;  /**< Human-readable informational text displaying syntax requirements and usage instructions */
};

/**
 * @brief Functional run configuration properties adjusting the local terminal console interface behavior.
 */
struct MShellSettings {
    bool echo = true; /**< Controls if raw characters are sent back over standard output streams during input typing */
};


/**
 * @brief Lightweight, non-allocating interactive shell terminal parsing engine for embedded rocket systems.
 */
class MShell {

    public:
    static constexpr uint8_t max_commands = 16;   /**< Limit defining the total amount of unique terminal commands allowed */
    static constexpr uint8_t max_args = 16;       /**< Maximum separate sub-argument tokens supported within one command line string */
    static constexpr uint8_t max_arg_length = 32; /**< Arbitrary individual string dimension bounds constraint metric */
    static constexpr uint8_t max_line_len = 255;  /**< Maximum text length boundaries processing continuous character buffers */

    MShellSettings settings; /**< Active internal visual display configuration state values */

    /**
     * @brief Maps a terminal string keyword onto a distinct program execution routing path.
     * @param name Unique trigger text string tracking incoming commands.
     * @param fn Executable function callback mapping system parameters.
     * @param help Informational text mapping string summaries detailing syntax targets.
     */
    void register_command(const char* name, MShellExecutor fn, const char* help = "") { 
        if (cmd_count_ >= max_commands) {
            return;
        }

        commands_[cmd_count_++] = { name, fn, help };
    }

    /**
     * @brief Tokenizes, extracts parameters, evaluates, and dispatches a complete raw input command sequence.
     * @param line Raw input string array collected across communications pathways.
     * @param with_sysarg Target runtime systems architecture configuration object references.
     * @return MCommandExecutionResult Functional status detailing matching implementation responses.
     */
    MCommandExecutionResult execute_line(const char* line, const void* with_sysarg) {
        char* argv_ptrs[max_args];
        uint8_t argc_ctx = 1;
        char l_cpy[max_line_len];
        strncpy(l_cpy, line, max_line_len - 1);
        l_cpy[max_line_len - 1] = '\0';

        char* cmd = strtok(l_cpy, " ");
        if(cmd == NULL) { return MCommandExecutionResult::ERR_INVALID_CMD; }
        argv_ptrs[0] = cmd;

        while(true) {
            char* arg_ptr = strtok(NULL, " ");
            if(arg_ptr == NULL) { break; }
            if(argc_ctx >= max_args) { break; }

            argv_ptrs[argc_ctx++] = arg_ptr;
        }
       
        MShellContext ctx = {argc_ctx, const_cast<const char**>(argv_ptrs), with_sysarg};
        int cmd_index = cmd_idx(cmd);

        if(cmd_index == -1) {
            // Command does not exist..
            return MCommandExecutionResult::ERR_NO_CMD;
        }

        MShellCommand& m_command = commands_.at(cmd_index);
        return m_command.fn(ctx);
    }

    /**
     * @brief Loops across registered structures to display a catalog of commands and instruction summaries.
     * @param out Target output destination interface stream.
     */
    void print_help(Stream* out) {
        for(int i = 0; i < cmd_count_; i++) {
            out->print(commands_[i].cmd);
            out->print(": ");
            out->println(commands_[i].help_text);
        }
    }

    private:
    /**
     * @brief Internal scan utility returning sequential positional database vectors for target terminal paths.
     * @param cmd_name Query string keyword identifier under validation evaluation.
     * @return int Array index of matching command definition entries, or -1 if no tracking register matches.
     */
    int cmd_idx(const char* cmd_name) {
        for(int i = 0; i < cmd_count_; i++) {
            MShellCommand c = commands_.at(i);
            if(strcmp(c.cmd, cmd_name) == 0) {
                return i;
            }
        }
        return -1;
    }

    uint8_t cmd_count_;                                      /**< Running tally tracking currently mapped functional definitions */
    std::array<MShellCommand, max_commands> commands_{};     /**< Fixed-size buffer array managing active instruction handles */
};

/**
 * @brief Top-level system lifecycle setup bootstrapping underlying operational parameters.
 */
void m_shell_setup();

/**
 * @brief Global instantiation reference managing command context handling globally.
 */
extern MShell m_shell_inst;

/**
 * @brief External functional callback parsing registration sequences to inject commands.
 * @param sh Target engine tracking runtime configuration allocations.
 */
void m_shell_init_commands(MShell* sh);