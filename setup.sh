ROS_PATH=/home/student/ros_ws/
DOC_PATH=/home/student/Documents/muffin
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
	exit
}

StartRobot()
{
	roslaunch baxter_gazebo baxter_world.launch
}

SetWorkSpace()
{
	cd $ROS_PATH
	./baxter.sh sim
	cp ~/Documents/muffin/setup.sh $ROS_PATH
}

case $1 in
	'sim')
		StartRobot
	;;
	'enable')
		EnableBaxter
	;;
	'sws')
		SetWorkSpace
	;;
	'disable')
		DisableBaxter
	;;
	'channels')
		MakeAch
	;;
	*)
		echo 'Commands'
		echo ' 	:sws - sets baxter workspace'
		echo '  :sim - starts sim robot'
		echo '  :enable - enables Baxter'
		echo '  :channels - makes ach channels'
		echo ' 	:disable - disables Baxter'
	;;
esac
