#pragma once
#include <cstdint>
#include <array>
#include <Arduino.h>
#include <string.h>

// Midas serial shell

enum class MCommandExecutionResult {
    // Generic
    OK = 0,                 // The command was successful
    ERR_UNSPECIFIED = 1,    // The execution returned an unspecified error
    ERR_INVALID_CMD = 2,    // The command given was not of a valid format
    ERR_NO_CMD = 3,         // The command given was not one that exists (but is of a valid format)
    ERR_INVAL_ARGC = 4,     // The amount of arguments is incorrect for the current function.
    ERR_INVAL_ARGUMENT = 5, // One or more arguments is invalid (generic)
    ERR_INVAL_ARG_RANGE = 6 // One or more arguments is out of range
    
    // Command specific
};

struct MShellContext {
    uint8_t argc;
    const char** argv;
    const void* sysarg = nullptr; // This will ALWAYS be RocketSystems*, but the type isn't available to us at the moment.
};

using MShellExecutor = MCommandExecutionResult(*)(const MShellContext&);

struct MShellCommand {
    const char* cmd;
    MShellExecutor fn;
    const char* help_text;
};

struct MShellSettings {
    bool echo = true; // Determines if on character entry, the shell returns the character entered (for visibility for human operators)
};


class MShell {

    public:
    static constexpr uint8_t max_commands = 16;
    static constexpr uint8_t max_args = 16;
    static constexpr uint8_t max_arg_length = 32;
    static constexpr uint8_t max_line_len = 255;

    MShellSettings settings;

    void register_command(const char* name, MShellExecutor fn, const char* help = "") { 
        if (cmd_count_ >= max_commands) {
            return;
        }

        commands_[cmd_count_++] = { name, fn, help };
    }

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

    void print_help(HWCDC* out) {
        // Prints command help to the provided HWCDC.
        for(int i = 0; i < cmd_count_; i++) {
            out->print(commands_[i].cmd);
            out->print(": ");
            out->println(commands_[i].help_text);
        }
    }

    private:
    // Returns a command's index in commands_
    int cmd_idx(const char* cmd_name) {
        for(int i = 0; i < cmd_count_; i++) {
            MShellCommand c = commands_.at(i);
            if(strcmp(c.cmd, cmd_name) == 0) {
                return i;
            }
        }
        return -1;
    }

    uint8_t cmd_count_;
    std::array<MShellCommand, max_commands> commands_{};
};

// Sets up the MIDAS shell
void m_shell_setup();
extern MShell m_shell_inst;