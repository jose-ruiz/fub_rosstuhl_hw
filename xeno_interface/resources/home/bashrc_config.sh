echo 'Importing ROSStuhl bash config'

source /opt/ros/kinetic/setup.bash

P="~/repositories"

export MY_ROS_WS="$P/xeno_ros":"${P}/fub_rosstuhl_path_following/"
export ROS_PACKAGE_PATH=${MY_ROS_WS}:${ROS_PACKAGE_PATH}




get_absolute_path() {
 python -c "import os.path as p; print p.abspath(p.expanduser('$1'))"
}

ros_ws_chain_restart() {
	SAVEIFS=$IFS
	IFS=":"
	current_path=`pwd`
	for ws in ${MY_ROS_WS}; do
		IFS=$SAVEIFS
		echo "Restarting Workspace", $ws		
		#		eval "abs_base_path=$ws"
		abs_base_path=`get_absolute_path "${ws}"`
		printf "\tAbsolute path: $abs_base_path\n"
		devel_path="${abs_base_path}/devel"
		build_path="${abs_base_path}/build"
		cd $abs_base_path
		printf "\tCleaning $devel_path and $build_path\n"
		catkin clean --yes
		echo "\tRunning catkin build...\n"
		catkin build
		echo "\tSourcing chained development environment\n"
		source $devel_path/setup.bash
		IFS=":"
		
	done
	IFS=$SAVEIFS
	

}

ros_ws_chain() {
	SAVEIFS=$IFS
	IFS=":"
	for ws in ${MY_ROS_WS}; do
		IFS=$SAVEIFS
		devel_path=`get_absolute_path "$ws/devel/setup.bash"`
		echo "Sourcing $devel_path"
		source ${devel_path}
		IFS=":"
	done
	IFS=$SAVEIFS
}


export ROS_IP=192.168.2.11
ros_ws_chain
#export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/repositories/third_party/src/viso2/
export PS1="\[\e]0;\u@\h: \W\a\]${debian_chroot:+($debian_chroot)}\u@\h:\W\$ "
alias mata='kill -KILL $(ps  | grep python | awk '"'{print \$1}')"

alias gotoda="cd ~/repositories/data-analysis/mulmodpan"
