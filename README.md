# Kugle-Misc
Code, libraries and other tools built for the Kugle robot project but does not fit in the other repositories, eg. log-processing, periphiral drivers/interfaces, startup scripts etc.

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
