ROS_PATH=/home/student/ros_ws/

ACH_CHAN_BAXTER_REF='ref_channel'
ACH_CHAN_BAXTER_STATE='state_channel'

MakeAch()
{
	ach -1 -C $ACH_CHAN_BAXTER_REF -m 10 -n 3000
	ach -1 -C $ACH_CHAN_BAXTER_STATE -m 10 -n 3000
}

EnableBaxter()
{
	rosrun baxter_tools_enable.py -e
}

DisableBaxter()
{
	rosrun baxter_tools_enable.py -d
}

StartRobot()
{
	roslaunch baxter_gazebo baxter_world.launch
}

SetWorkSpace()
{
	cd $ROS_PATH
	case $1 in
		'sim')
			./baxter.sh sim
		;;
		'real')
			./baxter.sh
		;;
		*)
		echo 'no baxter type defined'
		;;
	esac
}
Start()
{
	MakeAch
	SetWorkSpace $1
	StartRobot 
	sleep 20
	EnableBaxter
}
Stop()
{	
	DisableBaxter
}
case $1 in
	'start')
		Start
	;;
	'stop')
		Stop
	;;
	'real')
		
	*)
		echo 'Commands'
		echo '	:start - Enable Baxter'
		echo ' 	:real - starts real robot'
		echo '	:sim  - starts sim robot'
		echo ' 	:stop - disables Baxter'
	;;
esac

exit 0
