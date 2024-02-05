assert sys.version_info >= (3, 8), "At least Python 3.8 is required to run SILSIM"

Import("env")

env.Execute("${PYTHONEXE} -m pip install -r ${PROJECT_LIBDEPS_DIR}/mcu_silsim/TARS-Controls/Simulation/6DOF_RK4/requirements.txt")
env.Execute("${PYTHONEXE} ${PROJECT_LIBDEPS_DIR}/mcu_silsim/TARS-Controls/Simulation/6DOF_RK4/simulation/pysim.py")