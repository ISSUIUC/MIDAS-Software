@echo off
REM Generate hilsim packet
protoc -I=. --python_out=. hilsimpacket.proto
python nanopb_generator/nanopb_generator.py hilsimpacket.proto

python nanopb_generator/nanopb_generator.py rocketstate.proto
protoc -I=. --python_out=. rocketstate.proto

cp hilsimpacket_pb2.py ../../hilsim/hilsimpacket_pb2.py
cp rocketstate_pb2.py ../../hilsim/rocketstate_pb2.py
REM Get the size of the packets and automatically dump it to a header file
