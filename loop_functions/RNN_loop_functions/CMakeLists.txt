link_directories(${CMAKE_BINARY_DIR}/controllers/footbot_rnn)

add_library(RNN_loop_functions SHARED
 rnn_loop_function.cpp
 rnn_loop_function.h
 rnn_qtuser_function.cpp
 rnn_qtuser_function.h)

target_link_libraries(RNN_loop_functions
  footbot_rnn
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  ${GALIB_LIBRARIES})

