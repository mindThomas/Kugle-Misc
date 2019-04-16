# Kugle-Misc
Code, libraries and other tools built for the Kugle robot project but does not fit in the other repositories, eg. log-processing, periphiral drivers/interfaces, startup scripts etc.

Most importantly this repository includes the real-time test code for the MPC library including obstacle avoidance.

# How to build
```bash
mkdir build
cd build
cmake ..
make
```

# ACADO dependency
To build and run the `mpc_code_generation` executable [ACADO](http://acado.github.io/) is needed. If not already installed, ACADO Toolkit will automatically be downloaded and compiled by running `cmake`, however ACADO will only be stored in `/tmp` why it will be removed if the PC is restarted.

Alternatively it is recommended to download, compile and install ACADO manually into your home folder:
```bash
cd ~
git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
cd ~/ACADOtoolkit
mkdir build
cmake -DCMAKE_BUILD_TYPE="Release" ..
echo 'source acado_env.sh' >> ~/.bashrc 
source acado_env.sh
```

# MPC simulation
This repository includes tests for the shape-accelerated model predictive controller including obstacle avoidance. The MPC can be simulated (faster than real-time) with a predefined trajectory and visualized with a 2D top-down view. Random obstacles will be generated during simulation. Run the simulation from the `build` folder after compiling.
```bash
./MPC_Test
```
