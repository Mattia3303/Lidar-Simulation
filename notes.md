# TMP Notes for simulating a Lidar sensor with noise

## Steps
1) **Installa gz-common (??)**
    E' necessaria per usare i sensori
    [cmp]
2) **Installa la repo gz-sensor**
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update


    sudo apt install libgz-sensors<#>-dev **6 e 9**


3) **Clone e Builda la repo**
git clone https://github.com/gazebosim/gz-sensors

cd gz-sensors; mkdir build; cd build; cmake ..;  makeù


4) **modifica la variabile d'ambiente di gazebo in modo che veda il plugin**
cd ./gz-sensor
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build/lib






