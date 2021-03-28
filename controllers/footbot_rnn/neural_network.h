#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

#include <argos3/core/control_interface/ci_controller.h>

using namespace argos;

class CNeuralNetwork {

public:

   CNeuralNetwork();
   virtual ~CNeuralNetwork();

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();
   virtual void Destroy();

   void ShowInput();

   virtual void LoadNetworkParameters(const UInt32 un_num_params,
                                      const Real* pf_params) = 0;

   void SetInput(UInt32 un_input_num,
                 Real f_input_value) {
      m_pfInputs[un_input_num] = f_input_value;
   }

   inline const Real* GetInputs() {
      return m_pfInputs;
   }

   inline  const Real* GetOutputs() {
      return m_pfOutputs;
   }
   inline  Real GetOutput(UInt32 un_num_output) {
      return m_pfOutputs[un_num_output];
   }

   virtual void SetOnlineParameters(const UInt32 un_num_params,
                                    const Real* pf_params);

   protected:

   UInt32 m_unNumberOfInputs;
   UInt32 m_unNumberOfOutputs;

   Real* m_pfInputs;
   Real* m_pfOutputs;

   std::string m_strParameterFile;
};

#endif