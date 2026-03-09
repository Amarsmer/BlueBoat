#!/usr/bin/env python3

import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import math
import random

class NN(nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(NN, self).__init__()
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        
        # Define layers
        self.hidden = nn.Linear(input_size, hidden_size)
        self.output = nn.Linear(hidden_size, output_size)
        
        # Activation function
        self.activation = nn.Tanh()
        
        # Initialize weights with uniform distribution [-1, 1]
        nn.init.uniform_(self.hidden.weight, -1.0, 1.0)
        nn.init.uniform_(self.output.weight, -1.0, 1.0)
        nn.init.zeros_(self.hidden.bias)
        nn.init.zeros_(self.output.bias)
        
    def forward(self, x):
        # Convert in pytorch if necessary
        if not isinstance(x, torch.Tensor):
            x = torch.tensor(x, dtype=torch.float32)
            
        # Forward pass through layers
        x = self.activation(self.hidden(x))
        x = self.activation(self.output(x))
        return x
    
    def run_nn(self, inputs):
        with torch.no_grad():
            x = torch.tensor(inputs, dtype=torch.float32)
            outputs = self.forward(x)
            return outputs.tolist()
    
    def load_weights_from_json(self, json_obj):
        # Load weights from a JSON file
        hidden_size = len(json_obj["input_weights"][0][:])

        # Convert input weights in pytorch tensors
        input_weights = torch.zeros(self.input_size, hidden_size)
        for i in range(self.input_size):
            for j in range(hidden_size):
                input_weights[i][j] = json_obj["input_weights"][i][j]
        
        # Convert output weights in pytorch tensors
        output_weights = torch.zeros(hidden_size, self.output_size)
        for i in range(hidden_size):
            for j in range(self.output_size):
                output_weights[i][j] = json_obj["output_weights"][i][j]
        
        # Apply weights to model
        with torch.no_grad():
            self.hidden.weight.copy_(input_weights.t())
            self.output.weight.copy_(output_weights.t())
            
    def save_weights_to_json(self):
        # Save weights in JSON file
        # Convert input weights in original format
        input_weights = []
        hidden_weights = self.hidden.weight.detach().t()
        for i in range(self.input_size):
            row = []
            for j in range(self.hidden_size):
                row.append(float(hidden_weights[i][j]))
            input_weights.append(row)
        
        # Convert output weights in original format
        output_weights = []
        out_weights = self.output.weight.detach().t()
        for i in range(self.hidden_size):
            row = []
            for j in range(self.output_size):
                row.append(float(out_weights[i][j]))
            output_weights.append(row)
        
        return {"input_weights": input_weights, "output_weights": output_weights}