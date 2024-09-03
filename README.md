# UT RoboMaster Software

This repository contains the code for the software sub-team for UT RoboMaster. The software sub-team is one of the two
software-focused teams in RoboMaster, the other being the firmware sub-team. The distinction between the two teams can
be roughly understood as firmware handles the direct control of the robot, while software runs on a separate board to
try to provide more information or autonomy to the robot and operators.

## Installation

### Pre-requisites

For any platform, you will need [Git](https://git-scm.com/downloads), [Docker](https://www.docker.com/), and ideally [Visual Studio Code](https://code.visualstudio.com/download), but if you strongly prefer you can replace Visual Studio Code with an IDE of your preference.

#### Windows

If you are on a Windows platform, keep in mind that you will not be able to use the built in command line for Windows, as it does not support shell scripts. Instead, you should either opt to use Git Bash, which comes installed with Git by default, or if you are more tech-savvy, feel free to attempt to use Windows Subsystem Linux (scripts are untested on WSL).

Additionally, when you clone this repo, try to make sure there are no spaces in the path to the repo, as that can potentially break some scripts.

### Cloning the Repo

#### SSH Authentication Setup

If you have already setup git with SSH authentication you may skip this step.

Generate a SSH key in Terminal (MacOS, Linux) or Git Bash (Windows)

```
ssh-keygen -t ed25519 -C "your_email@example.com"
```

When prompted, just press enter until the command finishes. This will create an ssh key in the default location without a passkey. (You can choose a passkey if you would like, but it will ask you for the passkey every time you pull or push).

Copy the output of the following command:
```
cat ~/.ssh/id_rsa.pub
```

Open [GitHub -> Settings -> Keys](https://github.com/settings/keys) in a web browser.

Click "New SSH Key" and paste the output you copied earlier into the box titled "Key."

Click "Add SSH Key" to finish.

#### Cloning

Go to the directory that you want to clone the repo in (if on Windows, make sure it has no spaces). Then run:

```
git clone git@github.com:ut-ras/robomaster_cv.git
```