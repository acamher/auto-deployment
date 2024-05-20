# Auto-deployment

## Initialize

### ROS

```bash
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh
```

### MAVROS

```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

### SITL

```bash
sudo apt-get update
sudo apt-get install git
sudo apt-get install gitk git-gui
git config --global url."https://".insteadOf git://
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### JUPYTER

```bash
pip install notebook
```

### CONFIGURE DISPLAY

```bash
sudo nano ~/.bashrc
export DISPLAY=:0.0
```

### FIREWALL FOR WSL

```bash
netsh interface portproxy add v4tov4 listenport=14550 listenaddress=0.0.0.0 connectport=14550 connectaddress=172.26.144.1
```

### EN UBUNTU

```bash
hostname -I   ---> Para ver la ipUbuntu
```

### EN SITL

```bash
output add ipUbuntu:14551
output #for checking
```

### IN OTHER TEMRINAL

```bash
roslaunch mavros apm.launch fcu_url:=udp://:14551@ipUbuntu:14500
```

## How the system works

