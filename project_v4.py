import time 
from enum import Enum
import numpy as np
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID

class state(Enum):
	MANUAL = 0
	ARMING = 1
	TAKEOFF = 2
	WAYPOINT_1 =3
	WAYPOINT_2 = 4
	WAYPOINT_3 = 5
	WAYPOINT_4 = 6
	LANDING = 7
	DISARMING = 8

class backyard_flyer(Drone):
	"""docstring for backyard_flyer"""
	def __init__(self, connection):
		super().__init__(connection)
		self.move_north = 10
		self.move_east = 10
		self.east = 0
		self.north = 0
		self.altitude = 10
		self.waypoint = np.array([0.0, 0.0, 0.0, 0.0])
		self.coordinate = np.array([0.0, 0.0, 0.0])
		self.settle_time = 2  
		self.home = np.array([0.0, 0.0, 0.0])
		self.flight_state = state.MANUAL

		self.register_callback(MsgID.STATE, self.state_callback)
		self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
		self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)


	def state_callback(self):
		if self.flight_state == state.MANUAL:
			self.arming_transition()
		elif self.flight_state == state.ARMING:
			self.takeoff_transition()
		elif self.flight_state == state.LANDING:
			self.landing_transition()
		# elif self.flight_state ==state.DISARMING:
		# 	disarming_transition()

	def velocity_callback(self):
		if self.flight_state == state.LANDING:
			if (self.global_home[2] - self.global_position[2]) < 0.1 and self.local_position[2]*-1 < 0.01:
				self.disarming_transition()



	def local_position_callback(self):


		
		# print(self.coordinate)
		if self.flight_state == state.TAKEOFF:

			alti = -1.0 * self.local_position[2]

			
			if alti > 0.95 * self.altitude:

				self.waypoint_transition()
			   
		
		

		# if self.flight_state.value in range(state.WAYPOINT_1.value,state.WAYPOINT_2.value+1):

		if self.flight_state == state.WAYPOINT_1:
			# print("WAYPOINT_1 = ",self.local_position)
			# self.controller()
			if ((self.coordinate[0] * 0.95) < abs(self.local_position[0])):
				time.sleep(self.settle_time)
			
				self.waypoint_transition()
			# print("state = ",self.flight_state)
		
		elif self.flight_state == state.WAYPOINT_2:
			# print("WAYPOINT_2 = ",self.local_position)
			# self.controller()
			if ((self.coordinate[1] * 0.95) < abs(self.local_position[1])):
				time.sleep(self.settle_time)
				self.waypoint_transition()
				# print("state = ",self.flight_state)


		# elif self.flight_state.value in range(state.WAYPOINT_3.value,state.WAYPOINT_4.value+1):
			# print("position = ", self.local_position)
			# print("state = ",self.flight_state)
		elif self.flight_state == state.WAYPOINT_3:
			# print("WAYPOINT_3 = ",self.local_position)
			# self.controller()
			if ((self.move_north * 0.05) > abs(self.local_position[0])):
				time.sleep(self.settle_time)
				self.waypoint_transition()
		elif self.flight_state == state.WAYPOINT_4:
			# print("WAYPOINT_4 = ",self.local_position)
			# self.controller()
			if ((self.move_east * 0.05) > abs(self.local_position[1])):
				time.sleep(self.settle_time)
				
				self.landing_transition()



	def waypoint_transition(self):
			print("waypoint_transition")
			
			if self.flight_state == state.TAKEOFF:
				self.waypoint = np.array([self.move_north, self.east, self.altitude,0])
				self.coordinate = np.array([self.waypoint[0], self.waypoint[1], self.waypoint[2]])
				# print("waypoint = ",self.waypoint)
				self.cmd_position(self.waypoint[0], self.waypoint[1], self.waypoint[2], self.waypoint[3])


				# time.sleep(5)

				self.north = self.move_north
				self.flight_state = state.WAYPOINT_1
				# print("1",self.waypoint)


			elif self.flight_state == state.WAYPOINT_1:
				self.waypoint = np.array([self.north, self.move_east, self.altitude,0])
				self.coordinate = np.array([self.waypoint[0], self.waypoint[1], self.waypoint[2]])
				self.cmd_position(self.waypoint[0], self.waypoint[1], self.waypoint[2], self.waypoint[3])
				self.east = self.move_east
				self.flight_state = state.WAYPOINT_2
				# print("2",self.waypoint)


			elif self.flight_state == state.WAYPOINT_2:
				self.waypoint = np.array([self.north - self.move_north, self.east, self.altitude, 0])
				self.coordinate = np.array([self.waypoint[0], self.waypoint[1], self.waypoint[2]])
				self.cmd_position(self.waypoint[0], self.waypoint[1], self.waypoint[2], self.waypoint[3])

				# time.sleep(5)


				self.north = self.north - self.move_north
				self.flight_state = state.WAYPOINT_3
				# print("3",self.waypoint)

			elif self.flight_state == state.WAYPOINT_3:
				self.waypoint = np.array([self.north, self.east - self.move_east, self.altitude, 0])
				self.coordinate = np.array([self.waypoint[0], self.waypoint[1], self.waypoint[2]])
				self.cmd_position(self.waypoint[0], self.waypoint[1], self.waypoint[2], self.waypoint[3])

				# time.sleep(5)
				
				self.east = self.east - self.move_east
				self.flight_state = state.WAYPOINT_4
				# print("4",self.waypoint)	


	def takeoff_transition(self):
		print("takeoff_transition")
		self.takeoff(self.altitude)
		self.flight_state = state.TAKEOFF

	def arming_transition(self):
		print("arming_transition")
		self.take_control()
		self.arm()
		self.home = np.array([self.global_position[0], self.global_position[1], self.global_position[2]])
		self.flight_state = state.ARMING

	def landing_transition(self):
		print("landing_transition")
		self.land()
		self.flight_state = state.LANDING

	def disarming_transition(self):
		print("disarming_transition")
		self.disarm()
		self.flight_state = state.DISARMING
		self.manual_transition()


	def manual_transition(self):
		print("manual_transition")
		self.release_control()
		
		self.stop()
		self.flight_state = state.MANUAL

	def start(self):
		self.start_log("Logs", "NavLog.txt")
		print("Start logging")
		super().start()
		self.stop_log()


if __name__ == "__main__":
	conn = MavlinkConnection('tcp:127.0.0.1:5760',threaded=False,PX4=False)
	drone = backyard_flyer(conn)
	time.sleep(2)
	drone.start()







