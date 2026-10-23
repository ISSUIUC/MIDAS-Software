#include "midas_shell.h"

/**
 * @brief Global instantiation of the MIDAS system command line engine.
 * @details Handles the operational parsing arrays, settings configuration, 
 * and callback routing for all terminal interactions.
 */
MShell m_shell_inst;

/**
 * @brief Terminal callback handle modifying console input visibility.
 * @details Toggles the character echo property on or off based on the parsed string arguments.
 * @param ctx Shell context containing target setting parameters.
 * argv[1][0]: character '1' to enable echo, '0' to disable it.
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult _c_echo(const MShellContext& ctx) {
    // Validate argument count: expects exactly 1 argument after the command keyword
    if(ctx.argc != 2) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    
    // Isolate the first character of the secondary parameter array element
    char n = ctx.argv[1][0]; // First argument (argv 1) will be 0/1 for the echo status, discard all other chars.
    
    // Evaluate operational switch criteria and assign the internal setting
    if(n == '1') { m_shell_inst.settings.echo = true; return MCommandExecutionResult::OK; }
    if(n == '0') { m_shell_inst.settings.echo = false; return MCommandExecutionResult::OK; }
    
    // Return error if input configuration state is unrecognized
    return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
}

/**
 * @brief Terminal callback to output the complete menu of registered shell options.
 * @details Flushes the text descriptions and helper documentation strings out to the system Serial line.
 * @param ctx Shell execution parameter context (unused).
 * @return MCommandExecutionResult Execution status code.
 */
MCommandExecutionResult _c_help(const MShellContext& ctx) {
    // Pipe all registered command mappings and help strings to the default Serial output hardware stream
    m_shell_inst.print_help(&Serial);
    return MCommandExecutionResult::OK;
}

void m_shell_setup() {
    // Sets up internal shell commands for the global shell instance
    
    // Map the built-in 'echo' terminal toggle option to its logic handler
    m_shell_inst.register_command("echo", _c_echo, "\t\techo <0|1> - Enable/disable 'echo' during character input. (Default 1)");
    
    // Map the standard 'help' index printing routing sequence
    m_shell_inst.register_command("help", _c_help, "\t\thelp - Prints this menu");
}