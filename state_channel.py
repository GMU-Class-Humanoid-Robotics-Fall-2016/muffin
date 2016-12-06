import ach
import time
import argparse
import sys
import rospy
import baxterStructure.py as bs

data_out = ach.Channel('ref_channel')
data_in  = ach.Channel('state_channel')

state = bs.STATE()
ref   = bs.STATE()

data_in.flush()
print("init node")
rospy.init_node("baxter_joint_pos_set")
right = bi.Limb('right')
left  = bi.Limb('left')
rate  = rospy.Rate(1000)


while True:
   [statuss, framesizes] = data_in.get(state,wait=False,last=False)
   bs.moveArm(ref,bs.RIGHT,right)
   bs.moveArm(ref,bs.LEFT,left)
   state = getState(state,ref,left,right)
   data_out.put(state)
   time.sleep(20)