# Multi-Robot-Search RNN
Multi-robot Search Based on Recurrent Neural Network(RNN)

The simulation platform used is Argos 3. So if you want to compile the project, please install argos3 first(https://github.com/ilpincy/argos3).

In this task, the robots need to find the path between the two specific areas in an unknown space, so that the multi-robot can quickly transport and place food in these two areas. I built a recurrent neural network (RNN) to train swarm robots to learn the cooperative search strategy. Meanwhile, I defined a virtual energy value for each robot to simulate the consumption of the robot in the exploration process. And construct the fitness function based on the energy consumption and the number of times the robot enters the two specific areas back and forth which enable swarm robots to carry out round-trip transportation more frequently in these two areas. The entire neural network is trained through the genetic algorithm (GA), and the exploration process from random to order during the continued training.

*** COMPILATION ***

In principle, you can compile your code however you want. In practice,
the help of a build system makes your work much easier. In ARGoS, and
in these examples, we use CMake. To compile the examples, open up a
shell, go to the directory where you unpacked the tar.bz2 file and
type:

Download this project to your workplace first

$ git clone https://github.com/Yandong-Luo/Multi-Robot-Search--RNN.git

$ cd Multi-Robot-Search--RNN

$ mkdir build

$ cd build

To produce debuggable code (slow), type:

$ cmake -DCMAKE_BUILD_TYPE=Debug ..

To produce fast but not debuggable code, type:

$ cmake -DCMAKE_BUILD_TYPE=Release ..

Finally, launch the compilation with the command:

$ make

$ cd ..

The RNN experiments are divided in two parts. The
command

$ build/embedding/rnn_main/rnn_pathformation

runs the RNN process with the GALib library. Then, the command

$ argos3 -c experiments/PF_RNN-trials.argos

allows one to test a specific neural network.

# Multi-robot
<div align=center><img src="https://user-images.githubusercontent.com/57821839/112744337-da663400-8fd1-11eb-9b0a-06af2323da07.gif"/></div>



# Single robot
<div align=center><img src="https://user-images.githubusercontent.com/57821839/112744361-11d4e080-8fd2-11eb-9e2f-a4060c464394.gif"/></div>


