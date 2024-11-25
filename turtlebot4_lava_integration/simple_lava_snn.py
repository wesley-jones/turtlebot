import numpy as np
from lava.magma.core.model.py import PyLoihiProcessModel
from lava.magma.core.run_conditions import RunSteps
from lava.magma.core.run_configs import Loihi1SimCfg
from lava.proc.dense.process import Dense
from lava.proc.lif.process import LIF

class SimpleLavaSNN:
    def __init__(self):
        # Initialize network layers
        self.input_layer = LIF(shape=(3,))  # 3 inputs for left, center, right
        self.hidden_layer = LIF(shape=(10,))  # Hidden layer for processing
        self.output_layer = LIF(shape=(3,))  # Output layer for left, forward, right

        # Define connections
        self.input_to_hidden = Dense(weights=np.random.rand(3, 10))  # Random weights
        self.hidden_to_output = Dense(weights=np.random.rand(10, 3))  # Random weights

        # Connect the layers
        self.input_layer.s_out.connect(self.input_to_hidden.s_in)
        self.input_to_hidden.s_out.connect(self.hidden_layer.s_in)
        self.hidden_layer.s_out.connect(self.hidden_to_output.s_in)
        self.hidden_to_output.s_out.connect(self.output_layer.s_in)

    def run(self, lidar_input):
        # Input data to SNN
        self.input_layer.a_in.assign(lidar_input)

        # Run the network for a few steps
        self.input_layer.run(condition=RunSteps(num_steps=10), run_cfg=Loihi1SimCfg())

        # Capture output
        output_spikes = self.output_layer.s_out.recv()
        return output_spikes  # Will be interpreted as movement commands

    def stop(self):
        self.input_layer.stop()
