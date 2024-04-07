A tool which can extract data written by TARS to its SD card into a much more usable JSON file.

Steps to use this wonderful software:

1. Install Python 3.10+.
2. Install the library Lark using `pip install lark-parser`. Make sure you are installing for the right Python.
3. Change your current working directory to the folder containing `LogParser` (so it should be the `MIDAS-Software` directory).
   This is a Python limitation, sadly. 
4. Run using `python -m LogParser <path-to-raw>`

By default, this will output a JSON file next to the raw file. If you want it
to output to CSV, add the argument `--data csv` (or `-d csv`). 
If you want it to output to a specific location, use `-o <location>`, and the output
will be inferred from the file suffix.

A common problem you may encounter is mismatched checksums. This is not
currently a hard error, but is very likely a problem nonetheless. To remedy this,
elsewhere on your computer, clone a new copy of `MIDAS-Software` and use `git checkout <revision>`
to make that copy of this repository be at the version of flight code that created
on the raw data in the first place. Then navigate into that `MIDAS-Software` repository
and find the `log_format.h` file, which should be at `MIDAS/src/log_format.h`. Note the absolute
path of the `log_format.h` file and supply it as an argument to the log parser by
adding the argument `--format <path>` where path is the absolute path you noted earlier.