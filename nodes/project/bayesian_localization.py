# -*- coding: utf-8 -*-
"""Bayesian_Localization.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1hTEojPM8jtMrlTCP9TT6K7rpEoEuPDfl
"""

import matplotlib.pyplot as plt

class bayesian_localization():
    def __init__(self):
        self.topological_map = {0: "green", 1: "orange", 2: "green", 3: "yellow", 4: "blue", 5: "green", 6: "orange", 7: "blue", 8: "blue", 9: "orange", 10: "yellow"}
        self.num_states = len(self.topological_map)
        self.control_model = {-1: [0.85, 0.10, 0.05], 0:[0.05, 0.90, 0.05], 1:[0.05, 0.10, 0.85]}
        self.colors = ["blue", "green", "yellow", "orange", "nothing"]
        self.measurement_model = {"blue": [0.6, 0.2, 0.05, 0.05, 0.1], "green": [0.2, 0.6, 0.05, 0.05, 0.1], "yellow":[0.05, 0.05, 0.65, 0.15, 0.1], "orange":[0.05, 0.05, 0.2, 0.6, 0.1]}
        self.probabilities_location = [[1.0/self.num_states for i in range(self.num_states)]] # p(x_k)
    

    def update(self, control, measurement):
        if(control == None or measurement == None):
            return True

        # === STATE PREDICTION ===
        curr_probabilities = self.probabilities_location[-1]
        pred_probabilities = [0 for i in range(self.num_states)]
        for i in range(0, self.num_states):
            for j in range(3):
                if (i-1 >= 0):
                    pred_probabilities[i] += self.control_model[control][2-j]*curr_probabilities[(i-1+j)%self.num_states]
                else:
                    pred_probabilities[i] += self.control_model[control][2-j]*curr_probabilities[i-1+j]

        # === STATE UPDATE ===
        for i in range(self.num_states):
            color_index = self.colors.index(measurement)
            pred_probabilities[i-2] *= self.measurement_model[self.topological_map[i]][color_index]

        # Normalization
        total = 0.0
        for i in pred_probabilities:
            total += i
        
        for i in range(len(pred_probabilities)):
            pred_probabilities[i] = pred_probabilities[i] / total
        self.probabilities_location += [pred_probabilities]

        return True

    def plot(self, output_graph = True):
        print("========== BAYESIAN LOCALIZATION ==========")
        count = 1
        plot_index = range(self.num_states)
        for i in self.probabilities_location:
            print("step ", count, " : ", i.index(max(i)), max(i))
            count += 1
            if (output_graph):
                plt.bar(plot_index, i)
                plt.show()
        return True

def move(robot, actions, output_graph = True):
    for i in actions:
        robot.update(i[0], i[1])
    robot.plot(output_graph)
    return True

actions = [[1, None], [1, "orange"], [1, "yellow"], [1, "green"], [1, "blue"], [1, "nothing"], [1, "green"], [1, "blue"], [0, "green"], [1, "orange"], [1, "yellow"], [1, "green"], [1, "blue"]]

# ==== MAIN FUNCTION ====

robot = bayesian_localization()

_ = move(robot, actions, True)