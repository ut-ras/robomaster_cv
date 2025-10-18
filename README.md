# UT RoboMaster Software

This repository contains the code for the software sub-team for UT RoboMaster. The software sub-team is one of the two
software-focused teams in RoboMaster, the other being the firmware sub-team. The distinction between the two teams can
be roughly understood as firmware handles the direct control of the robot, while software runs on a separate board to
try to provide more information or autonomy to the robot and operators.

# Installation

## Pre-requisites

For any platform, you will need [Git](https://git-scm.com/downloads), [Docker](https://www.docker.com/), and ideally [Visual Studio Code](https://code.visualstudio.com/download), but if you strongly prefer you can replace Visual Studio Code with an IDE of your preference.

### Windows

If you are on a Windows platform, keep in mind that you will not be able to use the built in command line for Windows, as it does not support shell scripts. Instead, you should either opt to use Git Bash, which comes installed with Git by default, or if you are more tech-savvy, feel free to attempt to use Windows Subsystem Linux (scripts are untested on WSL).

Additionally, when you clone this repo, try to make sure there are no spaces in the path to the repo, as that can potentially break some scripts.

## Cloning the Repo

### SSH Authentication Setup

If you have already setup git with SSH authentication you may skip this step.

Generate a SSH key in Terminal (MacOS, Linux) or Git Bash (Windows)

```
ssh-keygen -t ed25519 -C "your_email@example.com"
```

When prompted, just press enter until the command finishes. This will create an ssh key in the default location without a passkey. (You can choose a passkey if you would like, but it will ask you for the passkey every time you pull or push).

Copy the output of the following command:
```
cat ~/.ssh/id_ed25519.pub
```

Open [GitHub -> Settings -> Keys](https://github.com/settings/keys) in a web browser.

Click "New SSH Key" and paste the output you copied earlier into the box titled "Key."

Click "Add SSH Key" to finish.

### Cloning

Go to the directory that you want to clone the repo in (if on Windows, make sure it has no spaces). Then run:

```
git clone git@github.com:ut-ras/robomaster_cv.git
```

Once cloned, run:

```
cd robomaster_cv
```

Then, when finished, run:

```
git submodule update --init --recursive
```

## Dev Setup

Once the repo is cloned, open the repo in Visual Studio Code. Press `ctrl + grave` (grave is the button below the tilde, `~`) to open up the terminal pop-up.

If you are on Windows, make sure that it is running in Git Bash and not on command line. 

First, configure your environment by running
```
./configure.sh
```

This will set up the correct docker environment for you.

To enter the container, there are two methods, one involving VSC and one not. I recommend using VSC as it will enable syntax highlighting and smart language features.

### VSC

If you have VSC installed, install the Dev Containers extension and the Remote Development Extension pack.

Once installed, open the command palette (Ctrl + Shift + P or Cmd + Shift + P) and search for the option for "Reopen workspace in container". Wait for it to build and set up the environment, and you should be all set to go.

### Non-VSC

If you do not use VSC, run
```
./scripts/run_dev.sh
```

For every new terminal you wish to create, you should run this command in a new terminal.

### Git Setup (Optional)

Git is currently a bit funky with this system, and is not fully supported yet. If you wish to try to set this up, follow this [guide](https://code.visualstudio.com/remote/advancedcontainers/sharing-git-credentials) or try the instructions that worked for me:

1. Ensure that your sshd config enables agent forwarding. You should be able to find this file at `/etc/ssh/sshd_config` or `%programdata%\ssh\sshd_config`, and you will need to make sure it includes `AllowAgentForwarding yes`
2. Restart your sshd if needed. This will vary from system to system.
3. Open your environment in VSC (i.e. open it in the container)
4. On your host computer (not in VSC), open a terminal and run `ssh-add`
5. On the remote computer (inside VSC) and check that `ssh-add -l` contains a SSH key now

# Development

## Building Packages

The build system that ROS2 uses is called `colcon`. The repo includes several aliases to help with building.

To build a specific package, make sure you are in the root directory of the project (by default `/robomaster_cv` within the dev container) then run `build package_name`. This is an alias to `colcon build --symlink-install --packages-up-to package_name`. 

Try this by running `build realsense2_camera` (should take ~3 min)

You can also run `build_all`, but this is likely broken in the current version of the repository. 

## Coding

All of our source code is located within the `src` directory. You can feel free to poke around the [packages](http://wiki.ros.org/Packages), or check out the [software wiki](https://github.com/ut-ras/robomaster_cv/wiki) which goes more in depth.
