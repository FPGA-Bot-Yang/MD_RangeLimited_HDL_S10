# Compile library
cd ..
source gen_library.sh
cd test
echo "Library generated!\n"

# Compile host
make
echo "Host file compiled!\n"

# Compile for emulation
aoc -march=emulator -l ../RL_LJ_Evaluation.aoclib -L .. device/LJ.cl -o bin/LJ.aocx
echo "Emulation file compiled!\n"

# Run emulation
env CL_CONTEXT_EMULATOR_DEVICE_INTELFPGA=1 ./bin/host
echo "Emulation done!\n"