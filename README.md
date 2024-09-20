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

Then, when finished, run:

```
git submodule update --init --recursive
```

## Dev Setup

Once the repo is cloned, open the repo in Visual Studio Code. Press `ctrl + grave` (grave is the button below the tilde, `~`) to open up the terminal pop-up.

If you are on Windows, make sure that it is running in Git Bash and not on command line. 

In terminal, run:

```
./scripts/run_dev.sh
```

This will kick off a run to build our Docker container which contains our dev environment which will take ~5 minutes, and then open it up in the terminal after. In future runs, it will take ~20 seconds to reopen the container after it is built. If you're curious, you can find more information about Docker [here](https://docs.docker.com/get-started/).

If you need more terminal windows, you can open another terminal window and run the same script, which will attach to the same container.

# Development

## Building Packages

The build system that ROS2 uses is called `colcon`. The repo includes several aliases to help with building.

To build a specific package, make sure you are in the root directory then run `build package_name`. This is an alias to `colcon build --symlink-install --packages-up-to package_name`. 

Try this by running `build realsense2_camera` (should take ~3 min)

You can also run `build_all`, but this is likely broken in the current version of the repository. 

## Coding

All of our source code is located within the `src` directory. You can feel free to poke around the [packages](http://wiki.ros.org/Packages), or check out the [software wiki](https://github.com/ut-ras/robomaster_cv/wiki) which goes more in depth.
