# Multi-Robot-Search RNN
Multi-robot Search Based on Recurrent Neural Network(RNN)

The simulation platform used is Argos.

In this task, the robots need to find the path between the two specific areas in an unknown space, so that the multi-robot can quickly transport and place food in these two areas. I built a recurrent neural network (RNN) to train swarm robots to learn the cooperative search strategy. Meanwhile, I defined a virtual energy value for each robot to simulate the consumption of the robot in the exploration process. And construct the fitness function based on the energy consumption and the number of times the robot enters the two specific areas back and forth which enable swarm robots to carry out round-trip transportation more frequently in these two areas. The entire neural network is trained through the genetic algorithm (GA), and the exploration process from random to order during the continued training.

# Multi-robot
<div align=center><src="https://user-images.githubusercontent.com/57821839/112744337-da663400-8fd1-11eb-9b0a-06af2323da07.gif"/></div>



# Single robot
<p align="center">![Screencast 2020-05-08 23_44_27 00_00_51-00_00_59](https://user-images.githubusercontent.com/57821839/112744361-11d4e080-8fd2-11eb-9e2f-a4060c464394.gif)</p>

