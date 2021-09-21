WORK IN PROGRESS!!!  
_Landing not accurate._  
# QUADCOPTER PRECISION LANDING SIMULATION WITHOUT ROS

Let's test precision landing with gazebo+ardupilot+**python3.5**, yes python3.5 this is a convenient and faster way to do so because the goal is to load virtual gimbal cámera topic loaded in the example world **iris_arducopter_runway** of **ardupilot_gazebo** which deploya simulation of a drone who can be controlled by **QGroundControl**.  

_**NOTE:**  pygazebo uses deprecated trollius library was replaced by asyncio but some functions which are used by pygazebo where deprecated in python >3.6, seems that the correct way is using ROS._  
  
## Installation 
```bash
sudo apt install git gitk git-gui curl
```
### QGroundControl
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
Then logout and login again.  
  
In the folder you want to save QGC:  
```bash
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
sudo chmod +x ./QGroundControl.AppImage
```
### Ardupilot  
```bash
git clone https://github.com/ArduPilot/ardupilot  
cd ardupilot  
git submodule update --init --recursive  
Tools/environment_install/install-prereqs-ubuntu.sh -y  
```
**Reload the path (log-out and log-in to make permanent):**  
```
 ~/.profile  
 ```
 **(optional) Add autotest folder to path to exec sim_vehicle.py from any path**  
 At the end of  ~/.bashrc file:  
 ```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest  
export PATH=/usr/lib/ccache:$PATH  
 ```
### Gazebo  
**ON UBUNTU**, with ubuntu 20.04 will get gazebo 11.0.  
```bash
curl -sSL http://get.gazebosim.org | sh  
sudo apt-get install libgazebo11-dev
# IMPORTANT! move models to gazebo modelo folder
cp prec_land_simu/models/*  ~/.gazebo/models/
```
### ardupilot_gazebo  
```bash
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```
### Python3.5, and packages pygazebo, mavlink, opencv-contrib 
Installing Python3.5 without mess up some previous installed version  
```bash
sudo apt-get install build-essential checkinstall
sudo apt-get install libreadline-gplv2-dev libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev
cd /usr/src
wget https://www.python.org/ftp/python/3.5.9/Python-3.5.9.tgz
sudo tar xzf Python-3.5.9.tgz
cd Python-3.5.9
sudo ./configure --enable-optimizations
sudo make altinstall
```
Installing python packages.  
```bash
sudo python3.5 -m pip install -U pip
sudo python3.5 -m pip install tqdm pymavlink dronekit Pillow asyncio numpy PyYaml opencv-contrib-python
```
And without opencv using apriltag
```bash
sudo python3.5 -m pip install tqdm pymavlink dronekit Pillow asyncio numpy PyYaml apriltag
```
pygazebo main repository works just until python2.7, wil3 github user mase some changes to work until 3.5 python version   
```bash
git clone https://github.com/wil3/py3gazebo
cd py3gazebo
sudo python3 setup.py install
```
## Usage
In three different terminals  
### Excecute pygazebo world  
```bash
cd prec_land_simu/  
gazebo worlds/iris_arducopter_runway.world  
```
This executes a world with a drone with a gimbal camera looking down, manually add some tags, is recomended to use **unit_box** as brakground because it's white so the tag could be detected easier.    
### Excecute sim_vehicle.py (connect to ardupilot)
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris
```
### Excecute landing_script.py (precision algorithm) 
```bash
cd prec_land_simu_pid/  
python3.5 landing_script_pid.py
```
### Open QGroundControl.
If not automatic connection, create one udp connection localhost:14551.  

**LETS FLY!**  
## REF 
**Gazebo installation:** http://gazebosim.org/tutorials?tut=install_ubuntu  
**Python3.5 installation:** https://tecadmin.net/install-python-3-5-on-ubuntu/  
**Trollius deprecated package:** https://pypi.org/project/trollius/  
**Trollius to asyncio tool (actual way):** https://pypi.org/project/trollius-fixers/  
**QGC Installation:** https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html  
**pygazebo DOC:** https://pygazebo.readthedocs.io/en/latest/  
**pygazebo until python 2.7** https://pypi.org/project/pygazebo/  
**AWESOME LANDING PID!!!** https://github.com/nikv96/AutonomousPrecisionLanding 