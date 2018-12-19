cd ..
source gen_library.sh
cd test

aoc -march=emulator -l ../RL_LJ_Evaluation.aoclib -L .. device/LJ.cl -o bin/LJ.aocx

env CL_CONTEXT_EMULATOR_DEVICE_INTELFPGA=1 ./bin/host
