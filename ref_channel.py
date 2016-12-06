import ach
import time
import baxterStructure as bs

data_out = ach.Channel('state_Channel')
data_in  = ach.Channel('ref_channel')

state = bs.STATE()
ref = bs.STATE()

data_in.flush()

while True:
   [statuss, framesizes] = data_in.get(state,wait=True,last=True)
   print state.arm[bs.LEFT].joint[bs.WY2].pos
   print state.arm[bs.RIGHT].joint[bs.WY2].pos
   ref.arm[bs.RIGHT].joint[bs.WY2].ref = 2.0
   ref.arm[bs.LEFT].joint[bs.WY2].ref = 1.0
   data_out.put(ref)
   time.sleep(10)
