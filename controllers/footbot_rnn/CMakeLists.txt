add_library(footbot_rnn SHARED
  footbot_rnn_controller.h
  footbot_rnn_controller.cpp
  neural_network.h
  neural_network.cpp
  rnn.cpp
  rnn.h
)
target_link_libraries(footbot_rnn
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
