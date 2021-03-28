# Multi-Robot-Search RNN
Multi-robot Search Based on Recurrent Neural Network(RNN)


In this task, the robots need to find the path between the two specific areas in an unknown space, so that the multi-robot can quickly transport and place food in these two areas. I built a recurrent neural network (RNN) to train swarm robots to learn the cooperative search strategy. Meanwhile, I defined a virtual energy value for each robot to simulate the consumption of the robot in the exploration process. And construct the fitness function based on the energy consumption and the number of times the robot enters the two specific areas back and forth which enable swarm robots to carry out round-trip transportation more frequently in these two areas. The entire neural network is trained through the genetic algorithm (GA), and the exploration process from random to order during the continued training.
