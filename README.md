# Pollibee Project

This project tries to use a crazyflie to directly pollinate a flower

## Installation

> Note that if you are working from our private VM, these should already be setup and you should be able to skip forward to the [Usage section](#usage)

### Pollibee Project

This project relies on the aerostack2 system. I have created a fork to test any local crazyflie or our system related requirements.

If you are setting up from scratch then go to the [next section first](#from-scratch) before setting up this section.

In your home directory (could be anywhere else but all the paths below are for your home directory)

```
mkdir -p ~/aerostack2_ws/src
cd ~/aerostack2_ws/src
git clone https://github.com/mhl787156/aerostack2.git
git clone https://github.com/mhl787156/project_pollibee.git
```

This will create a ros2 workspace and place the two project repositories in it.

You will need to install the dependencies by running the following:

```
sudo apt install git python3-rosdep python3-pip python3-colcon-common-extensions tmux tmuxinator -y
```
And then going back into the root workspace
```
cd ~/aerostack2_ws
sudo rosdep init
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src
```

Then, enable the handy aerostack2 cli

```
echo 'export AEROSTACK2_PATH=$HOME/aerostack2_ws/src/aerostack2' >> $HOME/.bashrc
echo 'source $AEROSTACK2_PATH/as2_cli/setup_env.bash' >> $HOME/.bashrc
echo 'source $HOME/aerostack2_ws/install/setup.bash' >> $HOME/.bashrc
source ~/.bashrc
```

This will enable you to build the project from any folder using

```
as2 build
```

Now as2 should be installed, now go to the [usage instructions](#usage) section below

> Note that if you want to make changes and push to either `aerostack2` or `project_pollibee` it is recommended that you update the git url in each repository to use the ssh version, and add your ssh key to your github account. 
>
> ```
> git remote set-url origin git@github.com:mhl787156/project_pollibee.git
> git remote set-url origin git@github.com:mhl787156/aerostack2.git
>```
> 
> To generate a new key see [these github instructions](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)


### From scratch 

> This section applies if you are trying to setup from scratch

This project relies on ROS2 Humble (End-of-life 2027), Ignition Gazebo Fortress, and Ubuntu 22.04. This section goes over how to install these elements. 

#### Installing Ubuntu 22.04

Your options are 

1. Finding a spare machine that you can wipe and install Ubuntu on
2. Finding a machine that you would be willing to dual boot
3. Running a virtual machine
4. Running a container (Not supported yet as I havent made a container - also this requires gazebo which requires a GUI which is not ideal for containers apart from singularity containers)

For options 1 and 2, you will need to find a USB stick that is >4Gb and using a tool such as [Rufus](https://rufus.ie/en/) flash the [Ubuntu 22.04 ISO file](https://ubuntu.com/download/desktop) onto it. 

Once you have a flashed USB drive, you can insert that into the spare machine. On startup make sure to mash some combination of F2, F8 or F12 to go to the BIOS boot screen and select boot from USB. 

This will start up tp the Ubuntu installer on the USB drive where you can select what to do. Whether that is to wipe the machine, or in the advanced menu create a new partition for Ubuntu so you can dual boot (For Windows you will also need to shrink your primary partition). For more details [see a guide such as this one](https://www.onlogic.com/company/io-hub/how-to-dual-boot-windows-11-and-linux/). 

You can also download the ISO file and run it in a virtual machine program such as [VirtualBox](https://www.virtualbox.org/). 

> I have created and exported a virtual machine with everything already installed for you to use. See the private teams group. 

In virtualbox you can import an existing virtual machine. Once installed and you have the gui up, there is an option to import (orange arrow), in which you can select the existing exported virtual machine.
Once you have a virtual machine setup, you can simply start it. 

> Note: Since this project uses gazebo and is not the lightest workioad, you may want to give the VM more resources i.e. CPU, RAM and potetially video memory too. You can do this in the virtual machine settings. 

#### Installing ROS2 Humble

For this project we will be using ROS2 Humble. Full installation instructions are [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). But in short:

```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Install
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update
sudo apt update
sudo apt upgrade

# Install ROS2
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Auto Source in bashrc to have access to ros2 tools
echo 'source ~/opt/ros/huble/setup.bash' >> $HOME/.bashrc
```

#### Installing Ignition Gazebo Fortress

The recommended compatible gazebo version for Ubuntu 22.04 and Humble is Fortress where installation instructions are [here](https://gazebosim.org/docs/fortress/install_ubuntu). But in short:

```
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```
You can check succesful installation by using the `ign` cli command

With these three installed, you should be ready to run the pollibees project, see [the above instrctions to continue](#pollibee-project-1)

## Usage

### Running Crazyflie Simulation

The initial usage example follows the [aerostack2 crazyflie tutorials which are here](https://aerostack2.github.io/_02_examples/crazyflie/index.html). It would be recommended to have a look through the aerostack2 docs, but to get the simulation up you can run the following.

Make sure you have followed the aerostack2 and project pollibee build instructions, then source the relevant files and go into the project pollibee folder
```
source /opt/ros/humble/setup.bash # You don't need to run if its already in your ~/.bashrc
source ~/aerostack2_ws/install/setup.bash # You don't need to run if its already in your ~/.bashrc
```
```
cd ~/aerostack2_ws/src/project_pollibee
```

In the project there is a core run file written in bash, for basic tasks this avoids the user having to remember lots of ros2 commands. See the tutorial for more info and more options, but to spin up a single drone run:

```
./launch_as2.bash -s
```

This will startup a tmux session with multiple tabs to startup the different ROS elements. Tmux is one way to run multiple things at the same time. Each tab is essentially the same as running another terminal window and uses your bash configuration and home directory. (It's not in its own container or anything like that). 

> Selecting your terminal window, you can scroll through the different tabs by pressing `ctrl+b` then a number 0 to 5. The initial terminal window you can input stuff in is tab 5. This is useful if things arent working properly and you want to see what's going on, debug messages etc. 

In the tmux tab that first pops up (tab number 5) you can run the mission using

```
python3 mission.py -s
```

And follow the instructions on screen. With any luck the crazyflie (its really small you might have to zoom into gazebo) will takeoff and do something. 

> Note: as of 10/03/2024 the crazyflie model has not been properly tuned. If it's not flying properly, change the file `sim_config/world.json` line 5 (`model_type`) to use `quadrotor_base` instead of crazyflie and it will use one of aerostack2 internal quadcopter models for now. 

In this terminal you can also run ROS2 introspection using the ros2 cli tools such as 
```bash
# Introspect current state
ros2 topic list
ros2 service list
ros2 action list

# Interact with ros2 streams
ros2 topic pub <topic name> <topic_type> <topic payload in yaml format>
...
ros2 --help
```
As well as run any extra scripts etc. 

**IMPORTANT** To stop your simulation and get out of the tmux environment you must run the stop.bash file in the project_pollibots directory.
```
./stop.bash 
```
This can be run both inside tmux, and in any terminal in the right directory and performs a clean exit of the simulation. 

**YOU SHOULD ALWAYS USE THIS COMMAND TO STOP THE SIMULATION**. 

Closing Gazbo directly won't stop the simulation. Make sure you do not have more than one simulation going at the same time! 

### Developing The Application

The primary file defining the mission is `mission.py`. 

Aerostack2 provides a really handy python api for abstracting out interacting with the vehicles through code (`DroneInterface`).

We will want to update it to basically do what we want! 

> Note that they have a distinction between the vehicles in the arena and the objects in the arena. 

The `sim_config` folder contains all of the configuration files for running the simulation. The main one is `world.json` which defines what is spawned into the world and where. 

> New models must be setup and placed within the `aerostack2/as2_simulation_assets/as2_gazebo_assets/models` and then `as2 build` to install them. There should exist the `pollibee_flower` object which can be selected as `model_type`

The other configurations are for the different modules and plugins which perform the interaction of the drone and simualtor/real world. 

### Real Crazyflies

> **THIS HAS NOT BEEN TRIED YET** 

See the instructions in the [orignal project](https://aerostack2.github.io/_02_examples/crazyflie/project_crazyflie_gates/index.html#real-execution)

Please write up instructions when we have worked them out. 
