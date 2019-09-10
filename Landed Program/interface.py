from msgdev import MsgDevice,PeriodTimer
from math import *
import time 
class Interface(object):
	def __init__(self,sub_addr,ahrs_port,gnss_port,motor_port_write,voltage_port):
		self.dev=MsgDevice()
		self.dev.open()
		self.dev.sub_connect(sub_addr+':'+ahrs_port)
		self.dev.sub_add_url('ahrs.roll')
		self.dev.sub_add_url('ahrs.pitch')
		self.dev.sub_add_url('ahrs.yaw')
		self.dev.sub_add_url('ahrs.roll_speed')
		self.dev.sub_add_url('ahrs.pitch_speed')
		self.dev.sub_add_url('ahrs.yaw_speed')
		self.dev.sub_add_url('ahrs.acce_x')
		self.dev.sub_add_url('ahrs.acce_y')
		self.dev.sub_add_url('ahrs.acce_z')

		self.dev.sub_connect(sub_addr+':'+gnss_port)
		self.dev.sub_add_url('gps.time')
		self.dev.sub_add_url('gps.posx')
		self.dev.sub_add_url('gps.posy')
		self.dev.sub_add_url('gps.posz')
		self.dev.sub_add_url('gps.stdx')
		self.dev.sub_add_url('gps.stdy')
		self.dev.sub_add_url('gps.stdz')
		self.dev.sub_add_url('gps.satn')
		self.dev.sub_add_url('gps.hspeed')
		self.dev.sub_add_url('gps.vspeed')
		self.dev.sub_add_url('gps.track')

		self.dev.sub_connect(sub_addr+':'+voltage_port)
		self.dev.sub_add_url('voltage')

		self.dev.pub_bind('tcp://0.0.0.0:'+motor_port_write)


	def receive(self,*args):
		data=[]
		for i in args:
			data.append(self.dev.sub_get1(i))
		return data

	def Motor_send(self,left_motor,right_motor):
		self.dev.pub_set1('pro.left.speed',left_motor)
		self.dev.pub_set1('pro.right.speed',right_motor)


def sign(x):
	if x<0:
		return -1
	elif x==0:
		return 0
	else:
		return 1
	
def motor_1500(x):
	if x>1500:
		return 1500
	else:
		return x


def path_following(x,y,yaw_m):
	x=0.0-x
	y=y-21
	if yaw_m>0:
		yaw_m=pi-yaw_m
	else:
		yaw_m=-yaw_m-pi
	m=abs(y-1)
	n=abs(yaw_m)
	if x>60:#Àë°¶½ÏÔ¶ Í£Ö¹
		n_left=0
		n_right=0
	elif n<=0.02 and m<=0.1:
		n_right = 1000
		n_left = 1000
	elif m>0.1:
		n_left =4*ceil(sin(0.5*m**0.5)*(280+sign(y-1)*70*(1/m**0.5)*cos((yaw_m*sign(y-1)-pi)/4)))
	else:
		n_left =2*ceil(cos(0.5 * n + 0.7) * (500 +300*n**0.5)*sign(yaw_m))
		n_right = 2*ceil(cos(0.5 * n + 0.7) * (500-300*n**0.5)*sign(yaw_m))
	return -motor_1500(n_left),motor_1500(n_right)

if __name__=="__main__":
	sub_addr='tcp://192.168.1.150'
	ahrs_port='55005'
	gnss_port='55004'
	motor_port_write='55002'
	voltage_port='55006'
	interface=Interface(sub_addr,ahrs_port,gnss_port,motor_port_write,voltage_port)
	t=PeriodTimer(0.1)
	t.start()
	try:
		while True:
			with t:
				data=interface.receive('gps.posx','gps.posy','ahrs.yaw','voltage')
				# left_motor,right_motor=path_following(data[0],data[1],data[2])
				print('posx:{},posy:{},yaw:{},voltage:{}'.format(data[0],data[1],data[2],data[3]))
				# print('posx:{},posy:{},yaw:{},left:{},right:{}'.format(data[0],data[1],data[2],left_motor,right_motor))
				interface.Motor_send(500,500)
	except (KeyboardInterrupt, Exception) as e:
		# for i in range(20):
		interface.Motor_send(0,0)
		time.sleep(0.1)
		interface.dev.close()
		print('dev closed')
		raise
	finally:
		pass