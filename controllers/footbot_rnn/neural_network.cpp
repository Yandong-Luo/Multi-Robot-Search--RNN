#include "neural_network.h"
#include <argos3/core/utility/string_utilities.h>

/****************************************/
/****************************************/

CNeuralNetwork::CNeuralNetwork() :
   m_unNumberOfInputs(0),
   m_unNumberOfOutputs(0),
   m_pfInputs(NULL),
   m_pfOutputs(NULL) {}

/****************************************/
/****************************************/

CNeuralNetwork::~CNeuralNetwork() {
   // delete[]回收由new[]产生的多个基本数据对象:
   if(m_pfInputs)  delete[] m_pfInputs;
   if(m_pfOutputs) delete[] m_pfOutputs;
}

/****************************************/
/****************************************/

void CNeuralNetwork::Init(TConfigurationNode& t_node) {
   // Get the number of inputs, and initialise the input vector
   try {
      GetNodeAttribute(t_node, "num_inputs", m_unNumberOfInputs);
      
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("missing number of inputs for the neural network controller.", ex);
   }

   m_pfInputs = new Real[m_unNumberOfInputs];
   ::memset(m_pfInputs, 0, sizeof(Real) * m_unNumberOfInputs);

   // Get the number of outputs, and initialise the output vector
   try {
      GetNodeAttribute(t_node, "num_outputs", m_unNumberOfOutputs);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("missing number of outputs for the neural network controller.", ex);
   }

   m_pfOutputs = new Real[m_unNumberOfOutputs];
   ::memset(m_pfOutputs, 0, sizeof(Real) * m_unNumberOfOutputs);

   // name of the parameter file
   try{
      GetNodeAttributeOrDefault( t_node, "parameter_file", m_strParameterFile, m_strParameterFile);
   }
   catch(CARGoSException& ex) {
      std::cout<<"do not exist the parameter file\n";
   }
}


/****************************************/
/****************************************/

void CNeuralNetwork::Reset() {
   // reset the input vector
   ::memset(m_pfInputs, 0, sizeof(Real) * m_unNumberOfInputs);
   // reset the output vector
   ::memset(m_pfOutputs, 0, sizeof(Real) * m_unNumberOfOutputs);
}


/****************************************/
/****************************************/

void CNeuralNetwork::Destroy() {
   if( m_pfInputs ) delete[] m_pfInputs;
   m_pfInputs = NULL;
   m_unNumberOfInputs = 0;

   if( m_pfOutputs ) delete[] m_pfOutputs;
   m_pfOutputs = NULL;
   m_unNumberOfOutputs = 0;
}


/****************************************/
/****************************************/

void CNeuralNetwork::SetOnlineParameters(const UInt32 un_num_params,
                                         const Real* pf_params ) {
   LoadNetworkParameters(un_num_params, pf_params);
}

void CNeuralNetwork::ShowInput()
{
   for(UInt32 i=0;i<13;i++)
   {
      if(i<8)
      {
         std::cout<<"Proximity Sensor:"<<i<<"value:"<<m_pfInputs[i]<<"\n";
      }
      else if(i==8)
      {
         std::cout<<"Ground_Sensor:"<<i<<"value:"<<m_pfInputs[i]<<"\n";
      }
      else{
         std::cout<<"Camera_Sensor:"<<i<<"value:"<<m_pfInputs[i]<<"\n";
      }
   }
}
/****************************************/
/****************************************/
