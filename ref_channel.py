import ach
import time
import baxterStructure as bs

data_out = ach.Channel('ref_channel')
data_in  = ach.Channel('state_channel')

state = bs.STATE()
ref = bs.STATE()

data_in.flush()

commands = [0.1,0.5,1.2,1.4]

i = 0
while True:
   [statuss, framesizes] = data_in.get(state,wait=False,last=False)
   t0 = time.time()
   print 'state:'
   print state.arm[bs.RIGHT].joint[bs.EP].pos
   print state.time
   ref.arm[bs.RIGHT].joint[bs.EP].ref = commands[i]
   ref.arm[bs.LEFT].joint[bs.EP].ref = commands[i]
   data_out.put(ref)
   t1 = time.time()
   t = t1 - t0
   i = (i + 1)%4
   if (t < 10):
      time.sleep(10-t)
   
