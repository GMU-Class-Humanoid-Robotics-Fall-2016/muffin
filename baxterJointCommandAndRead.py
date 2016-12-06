#!/usr/bin/env python
import argparse
import sys
import rospy
import baxter_interface as bi
import baxterStructure as bs

def main():
  print("init node")
  rospy.init_node("baxter_joint_pos_set")
  left  = bi.Limb('left')
  right = bi.Limb('right')
  rate = rospy.Rate(1000)
  state = bs.STATE()
  ref = bs.STATE()

 # for arm in range(0,bs.NUM_ARMS):
 #   for joint in range(0,bs.BAXTER_ARM_JOINTS_NUM):
 #     ref.arm[arm].joint[joint].ref = 0.0
 #     ref.arm[arm].joint[joint].pos = 0.0
 #     ref.arm[arm].joint[joint].tor = 0.0

  ref.arm[bs.RIGHT].joint[bs.WY2].ref = 2.0
  moveArm(ref, bs.RIGHT, right)
  state = getState(state,ref,left,right)

  ref.arm[bs.LEFT].joint[bs.WY2].ref = 1.0
  moveArm(ref, bs.LEFT, left)
  state = getState(state,ref,left,right)

  print left.joint_angle('left_w2')
  print state.arm[bs.LEFT].joint[bs.WY2].pos
  print state.arm[bs.LEFT].joint[bs.WY2].ref
  print right.joint_angle('right_e1')

if __name__ == '__main__':
  main()
