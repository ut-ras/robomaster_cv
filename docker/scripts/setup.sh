#!/bin/bash

shopt -s expand_aliases
alias sl='source /robomaster_cv/install/local_setup.sh'
alias build="colcon build --symlink-install --packages-up-to"
alias build_all="colcon build --symlink-install"
alias clean="rm -r build install log"
sl