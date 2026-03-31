#include <systems.h>

MCommandExecutionResult set_fsm(const MShellContext& ctx) {
    // fix this command cause it's stupid
    RocketSystems* arg = (RocketSystems*) ctx.sysarg;
    if(ctx.argc != 2) { return MCommandExecutionResult::ERR_INVAL_ARGC; }

    int st = atoi(ctx.argv[1]);
    arg->rocket_data.fsm_state.update((FSMState)st);
    return MCommandExecutionResult::OK;
}

void m_shell_init_commands(MShell* sh) {
    sh->register_command("fsm", set_fsm, "fsm <state:int> - Sets the FSM state to state <state>");
}