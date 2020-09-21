import arcade
import pickle
import sys
from Simulator import SimState, SimStates, Polynomial

class SimWindow(arcade.Window):
	def __init__(self, filepath):
		super().__init__(400, 400, "Test Replay")
		arcade.set_background_color(arcade.color.WHITE)
		self.step = 0
		with open(filepath, 'rb') as file:
			self.data = pickle.load(file)
		self.set_update_rate(1 / 20)
		self.paused = True
		uavPoly = Polynomial(self.data.uavPolyRaw)
		print(f"uav poly: {uavPoly}")
		targetPoly = Polynomial(self.data.targetPolyRaw)
		print(f"target poly: {targetPoly}")

	def setup(self):
		""" Set up the game and initialize the variables. """
		pass

	def on_draw(self):
		"""
		Render the screen.
		"""
		arcade.start_render()
		currentState = self.data.states[self.step] #get the state to display
		for uav in currentState.uavs:
			arcade.draw_circle_filled((uav.pos[0] + 20) * 10, (uav.pos[1] + 20) * 10, 8, arcade.color.BLACK)

		for target in currentState.targets:
			arcade.draw_circle_filled((target.pos[0] + 20) * 10, (target.pos[1] + 20) * 10, 8, arcade.color.RED)

		arcade.draw_text(f"Time: {round(currentState.time,2)}",
			 0, self.width, arcade.color.BLACK, 11, anchor_x="left", anchor_y="top")
		arcade.draw_text(f"Info: {round(currentState.infoScore,2)}",
			 0, self.width - 10, arcade.color.BLACK, 11, anchor_x="left", anchor_y="top")

		#draw comms lines between uavs
		for i in range(len(currentState.uavs)):
			for j in range(i+1, len(currentState.uavs)):
				uav1 = currentState.uavs[i]
				uav2 = currentState.uavs[j]
				dx = uav1.pos[0]-uav2.pos[0]
				dy = uav1.pos[1]-uav2.pos[1]
				if dx**2 + dy**2 < self.data.commsDistance**2:
					#the uavs can communicate, draw a line to represent that
					arcade.draw_line((uav1.pos[0] + 20) * 10, (uav1.pos[1] + 20) * 10, (uav2.pos[0] + 20) * 10, (uav2.pos[1] + 20) * 10, arcade.color.GRAY, 3)

		for uav in currentState.uavs:
			for target in currentState.targets:
				dx = uav.pos[0]-target.pos[0]
				dy = uav.pos[1]-target.pos[1]
				if dx**2 + dy**2 < self.data.commsDistance**2:
					#the uavs can communicate, draw a line to represent that
					arcade.draw_line((uav.pos[0] + 20) * 10, (uav.pos[1] + 20) * 10, (target.pos[0] + 20) * 10, (target.pos[1] + 20) * 10, arcade.color.GREEN, 3)


		if self.step < len(self.data.states) - 1 and not self.paused:
			self.step += 1


	def on_update(self, delta_time):
		""" Movement and game logic """
		pass

	def on_key_press(self, key, modifiers):
		"""Called whenever a key is pressed. """

		if key == arcade.key.SPACE:
			self.paused = not self.paused


if __name__ == '__main__':
	window = SimWindow(sys.argv[1])
	arcade.run()