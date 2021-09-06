PS1='${debian_chroot:+($debian_chroot)}\[\033[01;36m\]\u@\[\033[01;31m\][DOCKER]\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \$ '
source /opt/ros/foxy/setup.bash
[[ -f /rep/install/setup.bash ]] && source /rep/install/setup.bash
export PYTHONPATH=$PYTHONPATH:$REP_ROOT
