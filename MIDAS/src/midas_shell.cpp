#include <midas_shell.h>

MShell m_shell_inst;

MCommandExecutionResult _c_echo(const MShellContext& ctx) {
    if(ctx.argc != 2) { return MCommandExecutionResult::ERR_INVAL_ARGC; }
    char n = ctx.argv[1][0]; // First argument (argv 1) will be 0/1 for the echo status, discard all other chars.
    if(n == '1') { m_shell_inst.settings.echo = true; return MCommandExecutionResult::OK; }
    if(n == '0') { m_shell_inst.settings.echo = false; return MCommandExecutionResult::OK; }
    return MCommandExecutionResult::ERR_INVAL_ARGUMENT;
}

MCommandExecutionResult _c_help(const MShellContext& ctx) {
    m_shell_inst.print_help(&Serial);
    return MCommandExecutionResult::OK;
}

void m_shell_setup() {
    // Sets up internal shell commands for the global shell instance
    m_shell_inst.register_command("echo", _c_echo, "echo <0|1> - Enable/disable 'echo' during character input. (Default 0)");
    m_shell_inst.register_command("help", _c_help, "help - Prints this menu");
}


    
