@echo off
REM Generate hilsim packet
protoc -I=. --python_out=. hilsimpacket.proto
python nanopb_generator/nanopb_generator.py hilsimpacket.proto

REM Get the size of the packets and automatically dump it to a header file