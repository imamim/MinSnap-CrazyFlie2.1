ﺏ
# GammaSwarm   
 The 2022 Gamma Team Integrated Swarm System Repository

# Setup 
Firs create folder with name:
```
cd
mkdir catkin_ws
```

Download the package with git clone or from IDE.

```
cd catkin_ws
git clone https://github.com/ITU-GammaTeam/GammaSwarm.git
```

Then run the `catkin_make` command at main directory.

```
cd GammaSwarm
catkin_make
```

After make, install pybullet 
```
cd src/GammaSwarm/src/
pip3 install -e .
```

Go the main directory catkin_ws and do `catkin_make`
```
cd 
cd catkin_ws/GammaSwarm
catkin_make
```

Lastly add your setup.bash file your system .bashrc
```
gedit ~/.bashrc
```

Then if you don't source ROS do this:
```
source /opt/ros/noetic/setup.bash
```

And add this two line you last line of .bashrc file:
```
source /home/Your_Computer_username/catkin_ws/crazyswarm/ros_ws/devel/setup.bash
source /home/Your_Computer_username/catkin_ws/GammaSwarm/devel/setup.bash
```


# CrazySwarm Setup
On the system we are use the CrazySwarm currently. For installation follow these steps:
```
cd
cd catkin_ws
export CSW_PYTHON=python3
sudo apt install -y ros-noetic-tf ros-noetic-tf-conversions ros-noetic-joy
sudo apt install -y libpcl-dev libusb-1.0-0-dev
sudo apt install -y swig lib${CSW_PYTHON}-dev ${CSW_PYTHON}-pip

${CSW_PYTHON} -m pip install pytest numpy PyYAML scipy
${CSW_PYTHON} -m pip install vispy
${CSW_PYTHON} -m pip install matplotlib

sudo apt install -y ffmpeg

${CSW_PYTHON} -m pip install ffmpeg-python

git clone https://github.com/USC-ACTLab/crazyswarm.git
cd crazyswarm
./build.sh

cd ros_ws/src/crazyswarm/scripts
source ../../../devel/setup.bash
$CSW_PYTHON -m pytest
```
Then installation done! Please close the terminal and open again!

Configure all of the dependencies of PyBullet
# Dependencies
```
pip3 install shapely
pip3 install rtree
pip3 install plotly
```

## Starting System
For V1.0 of system, you can use this 1 line code (After Setup Process you can do this):

```
roslaunch GammaSwarm allRun.launch
```

If you want configure of mission parameter please refer this 2 code which that given relative path!

```
src/GammaSwarm/src/GammaSwarm/mainSystem/Parameters.py
src/GammaSwarm/src/GammaSwarm/simulationSystems/SimulationParameter.py
```

## Developing
Bu kısım Türkçe anlatılacaktır!

GammaSwarm System V1.0 için düzenlenebilinecek şeyler şunlardır:
1- Mission.py içerisindeki modeList değişkeni:
```
src/GammaSwarm/src/GammaSwarm/mainSystem/Mission.py
```
2- Görev fonksiyonlarının yerine getirilmesi için izlenmesi gereken aşamalar şunlardır!

-Modes.py içerisine modu çalıştıracak yeni bir fonksiyon eklenmelidir. Nasıl bir yapıya sahip olacağı 'takeOff' fonksiyonunda görülmektedir.
-Ardından bu fonksiyon içerisinde kullanılacak merkezcil ve dağıtık fonksiyonlar 'uavClass.py' ve 'MerkezcilClass.py' içerisine eklenmelidir.

## PyBullet 
https://github.com/utiasDSL/gym-pybullet-drones 

## PIP 
https://pip.pypa.io/en/stable/ 

# General Use
--> Use launch files to configure standardized packages 

--> Do not use any recursive or try-expect function 

--> After every Ros dependent change (msg,service vs.), do the catkin_make at main dir

--> Before every push, make sure that you did pull 


# General Structure
```bash
Gamma Swarm
├── CMakeLists.txt
├── launch
│   ├── allRun.launch
│   ├── systemLaunch.launch
│   └── tryRun.launch
├── msg
│   ├── example.msg
│   ├── FullCommand.msg
│   ├── FullState.msg
│   ├── FullTrajectoryCommand.msg
│   ├── TrajectoryCommand.msg
│   ├── UavCommand.msg
│   └── UavState.msg
├── package.xml
├── runSystem.sh
├── setup.py
├── src
│   ├── CITATION.cff
│   ├── GammaSwarm
│   │   ├── mainSystem
│   │   │   ├── enums.py
│   │   │   ├── eskiKod.py
│   │   │   ├── formaitons.py
│   │   │   ├── gammaSwarm.py
│   │   │   ├── gecici_Formation.py
│   │   │   ├── gecici_utils.py
│   │   │   ├── Initializer.py
│   │   │   ├── mainServer.py
│   │   │   ├── MerkezcilClass.py
│   │   │   ├── mission.py
│   │   │   ├── Modes.py
│   │   │   ├── Parameters.py
│   │   │   ├── README.md
│   │   │   ├── real_control.py
│   │   │   ├── real_executer.py
│   │   │   ├── State.py
│   │   │   └── UavClass.py
│   │   └── simulationSystems
│   │       ├── ControllerUtils.py
│   │       ├── environment.py
│   │       ├── obstacle.urdf
│   │       ├── simulation_executer.py
│   │       └── SimulationParameter.py
│   ├── LICENSE
│   ├── README.md
│   └── setup.py
└── srv
    ├── RealServiceMessage.srv
    └── ServiceMessage.srv
```
