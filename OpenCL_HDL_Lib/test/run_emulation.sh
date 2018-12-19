# Compile library
cd ..
source gen_library.sh
cd test

# Compile host
make

# Compile for emulation
aoc -march=emulator -l ../RL_LJ_Evaluation.aoclib -L .. device/LJ.cl -o bin/LJ.aocx

# Run emulation
env CL_CONTEXT_EMULATOR_DEVICE_INTELFPGA=1 ./bin/host
