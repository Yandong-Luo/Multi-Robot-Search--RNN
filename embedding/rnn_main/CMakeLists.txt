add_executable(rnn_pathformation main.cpp)

target_link_libraries(rnn_pathformation
  ${GALIB_LIBRARIES}
  footbot_rnn
  RNN_loop_functions
  argos3core_simulator)
