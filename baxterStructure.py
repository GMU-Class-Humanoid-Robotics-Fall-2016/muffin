#!/usr/bin/env python
from ctypes import *

BAXTER_ARM_JOINTS_NUM = 7
NUM_ARMS = 2
RIGHT = 0
LEFT = 1
SY = 0
SP = 1
SR = 2
EP = 3
WY = 4
WP = 5
WY2 = 6

class JOINTS(Structure):
   _pack_ = 1
   _fields_ = [("ref",c_double),
               ("pos",c_double),
               ("tor",c_double)]

class ARM(Structure):
   _pack_ = 1
   _fields_ = [("joint",JOINTS*BAXTER_ARM_JOINTS_NUM)]

class STATE(Structure):
   _pack_ = 1
   _fields_ = [("arm",ARM*NUM_ARMS),
               ("time",c_double)]

class JOINTOFFSET(Structure):
   _pack_ = 1
   _fields_ = [("thetaOff",c_double),
               ("thetaDir",c_double)]

class ARMOFFSET(Structure):
   _pack_ = 1
   _fields_ = [("joint", JOINTOFFSET*BAXTER_ARM_JOINTS_NUM)]

class OFFSET(Structure):
   _pack_ = 1
   _fields_ = [("arm", ARMOFFSET*NUM_ARMS)]

off = OFFSET()

off.arm[RIGHT].joint[SY].thetaOff = 0.78
off.arm[RIGHT].joint[SY].thetaDir = 1.0
off.arm[RIGHT].joint[SP].thetaOff = 0.0
off.arm[RIGHT].joint[SP].thetaDir = 1.0
off.arm[RIGHT].joint[SR].thetaOff = 0.0
off.arm[RIGHT].joint[SR].thetaDir = 1.0
off.arm[RIGHT].joint[EP].thetaOff = 1.57
off.arm[RIGHT].joint[EP].thetaDir = 1.0
off.arm[RIGHT].joint[WY].thetaOff = 0.0
off.arm[RIGHT].joint[WY].thetaDir = -1.0
off.arm[RIGHT].joint[WP].thetaOff = 0.0
off.arm[RIGHT].joint[WP].thetaDir = 1.0
off.arm[RIGHT].joint[WY2].thetaOff = 0.0
off.arm[RIGHT].joint[WY2].thetaDir = -1.0

off.arm[LEFT].joint[SY].thetaOff = -0.78
off.arm[LEFT].joint[SY].thetaDir = 1.0
off.arm[LEFT].joint[SP].thetaOff = 0.0
off.arm[LEFT].joint[SP].thetaDir = 1.0
off.arm[LEFT].joint[SR].thetaOff = 0.0
off.arm[LEFT].joint[SR].thetaDir = 1.0
off.arm[LEFT].joint[EP].thetaOff = 1.57
off.arm[LEFT].joint[EP].thetaDir = 1.0
off.arm[LEFT].joint[WY].thetaOff = 0.0
off.arm[LEFT].joint[WY].thetaDir = -1.0
off.arm[LEFT].joint[WP].thetaOff = 0.0
off.arm[LEFT].joint[WP].thetaDir = 1.0
off.arm[LEFT].joint[WY2].thetaOff = 0.0
off.arm[LEFT].joint[WY2].thetaDir = -1.0

def B2A(state):
   stateOut = STATE()
   for arm in range(0,NUM_ARMS):
      for joint in range(0,BAXTER_ARM_JOINTS_NUM):
         stateOut.arm[arm].joint[joint].ref = state.arm[arm].joint[joint].ref * off.arm[arm].joint[joint].thetaDir - off.arm[arm].joint[joint].thetaOff
         stateOut.arm[arm].joint[joint].pos = state.arm[arm].joint[joint].pos * off.arm[arm].joint[joint].thetaDir - off.arm[arm].joint[joint].thetaOff
   return stateOut

def A2B(state):
   stateOut = STATE()
   for arm in range(0,NUM_ARMS):
      for joint in range(0,BAXTER_ARM_JOINTS_NUM):
         stateOut.arm[arm].joint[joint].ref = (state.arm[arm].joint[joint].ref + off.arm[arm].joint[joint].thetaOff) * off.arm[arm].joint[joint].thetaDir
         stateOut.arm[arm].joint[joint].pos = (state.arm[arm].joint[joint].pos + off.arm[arm].joint[joint].thetaOff) * off.arm[arm].joint[joint].thetaDir
   return stateOut

def moveArm(ref2, arm, limb):
  ref = bs.A2B(ref2)
  if arm == bs.RIGHT:
     angles       = {'right_s0': ref.arm[bs.RIGHT].joint[bs.SY].ref,
                     'right_s1': ref.arm[bs.RIGHT].joint[bs.SP].ref, 
                     'right_w0': ref.arm[bs.RIGHT].joint[bs.WY].ref, 
                     'right_w1': ref.arm[bs.RIGHT].joint[bs.WP].ref, 
                     'right_w2': ref.arm[bs.RIGHT].joint[bs.WY2].ref, 
                     'right_e0': ref.arm[bs.RIGHT].joint[bs.SR].ref, 
                     'right_e1': ref.arm[bs.RIGHT].joint[bs.EP].ref}
     limb.move_to_joint_positions(angles)
  elif arm == bs.LEFT:
     angles       = {'left_s0': ref.arm[bs.LEFT].joint[bs.SY].ref,
                     'left_s1': ref.arm[bs.LEFT].joint[bs.SP].ref, 
                     'left_w0': ref.arm[bs.LEFT].joint[bs.WY].ref, 
                     'left_w1': ref.arm[bs.LEFT].joint[bs.WP].ref, 
                     'left_w2': ref.arm[bs.LEFT].joint[bs.WY2].ref, 
                     'left_e0': ref.arm[bs.LEFT].joint[bs.SR].ref, 
                     'left_e1': ref.arm[bs.LEFT].joint[bs.EP].ref}
     limb.move_to_joint_positions(angles)

def getState(state,ref,left,right):
  state.arm[bs.LEFT].joint[bs.SY].pos = left.joint_angle('left_s0')
  state.arm[bs.LEFT].joint[bs.SP].pos = left.joint_angle('left_s1')
  state.arm[bs.LEFT].joint[bs.WY].pos = left.joint_angle('left_w0')
  state.arm[bs.LEFT].joint[bs.WP].pos = left.joint_angle('left_w1')
  state.arm[bs.LEFT].joint[bs.WY2].pos = left.joint_angle('left_w2')
  state.arm[bs.LEFT].joint[bs.SR].pos = left.joint_angle('left_e0')
  state.arm[bs.LEFT].joint[bs.EP].pos = left.joint_angle('left_e1')

  state.arm[bs.RIGHT].joint[bs.SY].pos = right.joint_angle('right_s0')
  state.arm[bs.RIGHT].joint[bs.SP].pos = right.joint_angle('right_s1')
  state.arm[bs.RIGHT].joint[bs.WY].pos = right.joint_angle('right_w0')
  state.arm[bs.RIGHT].joint[bs.WP].pos = right.joint_angle('right_w1')
  state.arm[bs.RIGHT].joint[bs.WY2].pos = right.joint_angle('right_w2')
  state.arm[bs.RIGHT].joint[bs.SR].pos = right.joint_angle('right_e0')
  state.arm[bs.RIGHT].joint[bs.EP].pos = right.joint_angle('right_e1')

  state2 = bs.B2A(state)

  for arm in range(0,bs.NUM_ARMS):
    for joint in range(0,bs.BAXTER_ARM_JOINTS_NUM):
      state2.arm[arm].joint[joint].ref = ref.arm[arm].joint[joint].ref

  return state2