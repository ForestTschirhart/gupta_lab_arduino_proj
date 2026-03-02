import re,sys
f='~/.arduino15/packages/arduino/hardware/sam/1.6.12/variants/arduino_due_x/variant.cpp'
f=f.replace('~',__import__('os').path.expanduser('~'))
s=open(f).read()
vals = re.findall(r"ulTCChannel *= *([0-9]+)", s)
vals = [int(x) for x in vals]
channelToId=[0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8]
channelToChNo=[0,0,1,1,2,2,0,0,1,1,2,2,0,0,1,1,2,2]
used=set(vals)
for i in sorted(used):
    print(f"index {i}: peripheral ID = ID_TC0+{channelToId[i]} -> ID_TC{channelToId[i]}, Tc block chNo={channelToChNo[i]}")