"""Virtual serial only; never opens a physical device. Isolated ROS domain 94."""
import os,pty,select,struct,time,subprocess,binascii,signal
os.environ.update(ROS_DOMAIN_ID='94',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
for k in ['CYCLONEDDS_URI','FASTRTPS_DEFAULT_PROFILES_FILE']:os.environ.pop(k,None)
import rclpy
from std_msgs.msg import Float64
master,slave=pty.openpty(); device=os.ttyname(slave)
assert device.startswith('/dev/pts/')
log=open('/tmp/yax_brake_review/driver_test.log','w')
exe='/tmp/yax_brake_review/install/vesc_driver/lib/vesc_driver/vesc_driver_node'
proc=subprocess.Popen([exe,'--ros-args','-p','port:='+device,'-p','zero_speed_brake_current:=20.0','-p','speed_min:=-50000.0','-p','speed_max:=50000.0','-p','brake_min:=0.0','-p','brake_max:=100.0'],stdout=log,stderr=subprocess.STDOUT)
rclpy.init();node=rclpy.create_node('virtual_brake_test');pub=node.create_publisher(Float64,'commands/motor/speed',10)
buf=bytearray();packets=[]
def frame(payload):
 return bytes([2,len(payload)])+payload+struct.pack('>H',binascii.crc_hqx(payload,0))+bytes([3])
def pump(duration):
 end=time.monotonic()+duration
 while time.monotonic()<end:
  if proc.poll() is not None:raise RuntimeError('driver exited: '+open('/tmp/yax_brake_review/driver_test.log').read())
  rclpy.spin_once(node,timeout_sec=.005)
  if select.select([master],[],[],.005)[0]:buf.extend(os.read(master,8192))
  while len(buf)>=5:
   assert buf[0]==2,buf[:20]
   n=buf[1]
   if len(buf)<n+5:break
   payload=bytes(buf[2:2+n]);del buf[:n+5]
   if payload[0]==0:os.write(master,frame(bytes([0,5,3])+b'TEST\x00'+bytes(32)))
   if payload[0] in [6,7,8]:packets.append(payload)
try:
 pump(2.)
 for rpm,expected in [(4203.,(8,4203)),(0.,(7,20000)),(-4203.,(8,-4203)),(0.,(7,20000)),(4203.,(8,4203))]:
  packets.clear()
  for _ in range(6):pub.publish(Float64(data=rpm));pump(.04)
  actual=[(p[0],struct.unpack('>i',p[1:5])[0]) for p in packets]
  assert len(actual)>=3 and all(p==expected for p in actual),(rpm,actual)
  print('PASS',rpm,expected,'packets',len(actual),flush=True)
finally:
 node.destroy_node();rclpy.shutdown();proc.send_signal(signal.SIGINT)
 try:proc.wait(timeout=5)
 except subprocess.TimeoutExpired:proc.kill();proc.wait()
 os.close(master);os.close(slave);log.close()
