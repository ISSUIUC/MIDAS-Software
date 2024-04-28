# Stores the console printing for the GSS combiner to not clog up `main.py`.

HELP_OUTPUT = """
USAGE:
py ./main.py [options]
 
   --booster [source1],[source2],[etc..]      -> Selects which COM ports should be interpreted as a data stream from the booster stage
   --sustainer [source1],[source2],[etc..]    -> Selects which COM ports should be interpreted as a data stream from the sustainer stage
   --local   (or -l)                          -> Streams all data to 'localhost' for testing. (Same as --ip localhost)
   --no-log  (or -n)                          -> Will not log data to logfiles for this run
   --verbose (or -v)                          -> Prints all telemetry events to console
   --no-vis  (or -nv)                         -> Shows a visual display of all systems
   --ip [IP] (or -i [IP])                     -> Connects to a specific IP. (Overrides --local)
   --help    (or -h)                          -> Prints this menu
   --config [config] (or -c [config])         -> Uses an argument config defined in config.ini. Added on top of existing params.
   --no-rf                                    -> Does not overwrite feather frequencies on startup
"""