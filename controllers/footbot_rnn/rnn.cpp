#include "rnn.h"
#include <cmath>
#include <fstream>
#include <argos3/core/utility/string_utilities.h>

CRnn::CRnn() :
   m_pfInputToHiddenWeights(NULL),
   m_pfHiddenToHiddenWeights(NULL),
   m_pfHiddenToOutputWeights(NULL),
   m_pfInputToOutputWeights(NULL),
   m_pfOutputBiases(NULL),
   m_pfHiddenBiases(NULL),
   m_pfHiddenTime(NULL),
   m_pfHidden(NULL){}
// m_cWeightsBounds(-1.0f, 1.0f),
//    m_cBiasesBounds(-4.0f, 4.0f),
//    m_cTausBounds(-1.0f, 3.0f) {}

/****************************************/
/****************************************/

CRnn::~CRnn() {
   if(m_pfInputToHiddenWeights)  delete[] m_pfInputToHiddenWeights;
   if(m_pfHiddenToHiddenWeights) delete[] m_pfHiddenToHiddenWeights;
   if(m_pfHiddenToOutputWeights) delete[] m_pfHiddenToOutputWeights;
   if(m_pfInputToOutputWeights)  delete[] m_pfInputToOutputWeights;
   if(m_pfHidden)                delete[] m_pfHidden;
   if(m_pfOutputBiases)          delete[] m_pfOutputBiases;
   if(m_pfHiddenBiases)          delete[] m_pfHiddenBiases;
   if(m_pfHiddenTime)            delete[] m_pfHiddenTime;
//    if(m_pfHiddenDeltaStates)     delete[] m_pfHiddenDeltaStates;
//    if(m_pfHiddenStates)          delete[] m_pfHiddenStates;
}


/****************************************/
/****************************************/

void CRnn::Init( TConfigurationNode& t_node ) {
   
   ////////////////////////////////////////////////////////////////////////////////
   // First perform common initialisation from base class
   ////////////////////////////////////////////////////////////////////////////////
   CNeuralNetwork::Init( t_node );
   
   ////////////////////////////////////////////////////////////////////////////////
   // Load number of hidden nodes
   ////////////////////////////////////////////////////////////////////////////////
   try{
      GetNodeAttribute(t_node, "num_hidden", m_unNumberOfHiddenNodes);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Missing required tag <num_hidden> for CTRNN multi-layer initialisation", ex);
   }
   
   ////////////////////////////////////////////////////////////////////////////////
   // integration step
   ////////////////////////////////////////////////////////////////////////////////
//    GetNodeAttribute(t_node, "integration_step", m_fTimeStep);

   ////////////////////////////////////////////////////////////////////////////////
   // Load upper and lower bounds for weigths, biases and time constant
   ////////////////////////////////////////////////////////////////////////////////

   try{
      GetNodeAttribute(t_node, "weight_range", m_cWeightsBounds);
      // std::cout<<"w_min:"<<m_cWeightsBounds.GetMin()<<"w_MAx:"<<m_cWeightsBounds.GetMax()<<"\n";
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("cannot load the bounds of weight from file.", ex);
   }
   try{
      GetNodeAttribute(t_node, "bias_range",  m_cBiasesBounds);
      // std::cout<<"b_min:"<<m_cBiasesBounds.GetMin()<<"b_MAx:"<<m_cBiasesBounds.GetMax()<<"\n";
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("cannot load the bounds of bias from file.", ex);
   }
   try{
      GetNodeAttribute(t_node, "time_range",  m_cTimeBounds);
      // std::cout<<"t_min:"<<m_cTimeBounds.GetMin()<<"t_MAx:"<<m_cTimeBounds.GetMax()<<"\n";
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("cannot load the bounds of time from file.", ex);
   }


   ////////////////////////////////////////////////////////////////////////////////
   // check and load parameters from file
   ////////////////////////////////////////////////////////////////////////////////
   if(m_strParameterFile != "") {
      try {
         LoadNetworkParameters(m_strParameterFile);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("cannot load parameters from file.", ex);
      }
   }
}


/****************************************/
/****************************************/

// void CRnn::Reset() {
//    for(size_t i = 0; i < m_unNumberOfHiddenNodes; ++i) {
//     //   m_pfHiddenDeltaStates[i] = 0.0f;
//       m_pfHidden[i]      = 0.0f;
//    }
// }

/****************************************/
/****************************************/

void CRnn::Destroy() {
   m_unNumberOfHiddenNodes = 0;

   if( m_pfInputToHiddenWeights )  delete[] m_pfInputToHiddenWeights;
   m_pfInputToHiddenWeights = NULL;

   if( m_pfHiddenToHiddenWeights ) delete[] m_pfHiddenToHiddenWeights;
   m_pfHiddenToHiddenWeights = NULL;

   if( m_pfHiddenToOutputWeights ) delete[] m_pfHiddenToOutputWeights;
   m_pfHiddenToOutputWeights = NULL;

   if( m_pfOutputBiases )          delete[] m_pfOutputBiases;
   m_pfOutputBiases = NULL;

   if( m_pfHiddenBiases )          delete[] m_pfHiddenBiases;
   m_pfHiddenBiases = NULL;

   if( m_pfHiddenTime )          delete[] m_pfHiddenTime;
   m_pfHiddenTime = NULL;

   if( m_pfInputToOutputWeights )  delete[] m_pfInputToOutputWeights;
   m_pfInputToOutputWeights = NULL;

   if(m_pfHidden) delete[] m_pfHidden;
   m_pfHidden = NULL;
}

/****************************************/
/****************************************/

// void CRnn::ComputeOutputs( void )
// {
//     // Update wieght of hidden layer from inputs and other neuron from the Hidden layer:
//     for(UInt32 i=0;i<m_unNumberOfHiddenNodes;i++)
//     {
//         // From the Input
//         for( UInt32 j = 0; j < m_unNumberOfInputs; j++ ) {
//          // weight * Input
//          m_pfHidden[i] += m_pfInputToHiddenWeights[i * m_unNumberOfInputs + j] * m_pfInputs[j] ;
//         }
//         for(UInt32 k = 0; k < m_unNumberOfHiddenNodes; k++)
//         {
//             // weight * Input
//             m_pfHidden[i] += m_pfHiddenToHiddenWeights[i * m_unNumberOfHiddenNodes + k] * m_pfHidden[k] ;
//         }
//         // Apply the transfer function (sigmoid with output in [0,1])
//         m_pfHidden[i] = 2.0f / ( 1.0f + ::exp( -m_pfHidden[i]) ) -1.0;
//     }
//     // Update the outputs layer::
//     for( UInt32 i = 0; i < m_unNumberOfOutputs; i++ )
//     {
//         // Initialise to 0
//         m_pfOutputs[i] = 0.0f;
//         // From the Input
//         for( UInt32 j = 0; j < m_unNumberOfInputs; j++ ) {
//          // weight * Input
//          m_pfOutputs[i] += m_pfInputToOutputWeights[i * m_unNumberOfInputs + j] * m_pfInputs[j] ;
//         }
//         for(UInt32 k = 0; k < m_unNumberOfHiddenNodes; k++)
//         {
//             // weight * Input
//             m_pfOutputs[i] += m_pfHiddenToOutputWeights[i * m_unNumberOfHiddenNodes + k] * m_pfHidden[k] ;
//         }
//         // Apply the transfer function (sigmoid with output in [0,1])
//         m_pfOutputs[i] = 1.0f / ( 1.0f + ::exp( -m_pfHidden[i]) );
//     }
// }

/****************************************/
/****************************************/

void CRnn::ComputeOutputs( void )
{
   // Update the value of hidden layer from last time and Inputs:
   for( UInt32 j = 0; j < m_unNumberOfHiddenNodes; j++ ) {
      m_pfHidden[j] = m_pfHiddenTime[j]*m_pfHidden[j];

      Real input = 0.0;
      for(UInt32 i=0;i<m_unNumberOfInputs;i++)
      {
         input = input + m_pfInputToHiddenWeights[j * m_unNumberOfInputs + i]*m_pfInputs[i];
         // input = 
      }
      Real Hj = (Real(1.0)/( exp(-( input + m_pfHiddenBiases[j])) + 1.0 ));
      m_pfHidden[j] = m_pfHidden[j] + (1-m_pfHiddenTime[j])*Hj;
   }

   // Update the outputs layer::
   for( UInt32 i = 0; i < m_unNumberOfOutputs; i++ ) {
      // Initialise to 0
      m_pfOutputs[i] = 0.0f;

      // from the input
      for( UInt32 j = 0; j < m_unNumberOfInputs; j++ ) {
         // weight * Input
         m_pfOutputs[i] += m_pfInputToOutputWeights[i * m_unNumberOfInputs + j] * m_pfInputs[j] ;
      }
      // from hidden
      for(UInt32 k = 0; k < m_unNumberOfHiddenNodes; k++)
      {
         // weight * Hidden
         m_pfOutputs[i] += m_pfHiddenToOutputWeights[i * m_unNumberOfHiddenNodes + k] * m_pfHidden[k] ;
     }
     m_pfOutputs[i] = (Real(1.0)/( exp(-( m_pfOutputs[i] + m_pfOutputBiases[i])) + 1.0 ));
   }
}

/****************************************/
/****************************************/

void CRnn::LoadNetworkParameters(const std::string& str_filename) {

   // open the input file
   std::ifstream cIn(str_filename.c_str(), std::ios::in);
   if( !cIn ) {
      THROW_ARGOSEXCEPTION("Cannot open parameter file '" << str_filename << "' for reading");
   }

   // first parameter is the number of real-valued weights
   UInt32 un_length = 0;
   if( !(cIn >> un_length) ) {
      THROW_ARGOSEXCEPTION("Cannot read data from file '" << str_filename << "'");
   }

   // check consistency between paramter file and xml declaration
   UInt32 m_unNumberOfWeights =
      m_unNumberOfHiddenNodes * (m_unNumberOfInputs + 1)  +
      m_unNumberOfOutputs * (m_unNumberOfHiddenNodes + 1) +
      m_unNumberOfInputs * m_unNumberOfOutputs            +
      m_unNumberOfHiddenNodes;

   if( un_length != m_unNumberOfWeights ) {
      THROW_ARGOSEXCEPTION("Number of parameter mismatch: '"
                           << str_filename
                           << "' contains "
                           << un_length
                           << " parameters, while "
                           << m_unNumberOfWeights
                           << " were expected from the XML configuration file");
   }

   // create weights vector and load it from file
   Real* m_pfWeights = new Real[m_unNumberOfWeights];
   for(size_t i = 0; i < m_unNumberOfWeights; ++i) {
      if( !(cIn >> m_pfWeights[i] ) ) {
         delete[] m_pfWeights;
         THROW_ARGOSEXCEPTION("Cannot read data from file '" << str_filename << "'");
      }
   }
   // showWeightParameters(m_unNumberOfWeights,m_pfWeights);
   // load parameters in the appropriate structures
   LoadNetworkParameters(m_unNumberOfWeights, m_pfWeights);
   delete[] m_pfWeights;
}

void CRnn::showWeightParameters(const UInt32 un_num_params,const Real* params)
{
   for(size_t i=0;i<un_num_params;i++)
   {
      std::cout<<"parameters:"<<params[i]<<"\n";
   }
}

/****************************************/
/****************************************/

void CRnn::LoadNetworkParameters( const UInt32 un_num_params, const Real* params ) {
   // check consistency between paramter file and xml declaration
   UInt32 un_num_parameters =
      m_unNumberOfHiddenNodes * (m_unNumberOfInputs + 1)  +
      m_unNumberOfOutputs * (m_unNumberOfHiddenNodes + 1) +
      m_unNumberOfInputs * m_unNumberOfOutputs            +
      m_unNumberOfHiddenNodes;
      if(un_num_params != un_num_parameters) {
      THROW_ARGOSEXCEPTION("Number of parameter mismatch: '"
                           << "passed "
                           << un_num_params
                           << " parameters, while "
                           << un_num_parameters
                           << " were expected from the xml configuration file");
   }

   UInt32 unChromosomePosition = 0;

   // Input to hidden weights
   if( m_pfInputToHiddenWeights == NULL ) m_pfInputToHiddenWeights = new Real[m_unNumberOfInputs * m_unNumberOfHiddenNodes];
   for( UInt32 i = 0; i < m_unNumberOfInputs * m_unNumberOfHiddenNodes; i++ ) {
      // m_pfInputToHiddenWeights[i] = params[unChromosomePosition++];
      m_pfInputToHiddenWeights[i] = params[unChromosomePosition++]*(m_cWeightsBounds.GetMax() - m_cWeightsBounds.GetMin() ) + m_cWeightsBounds.GetMin();
   }

   // Hidden Biases
   if( m_pfHiddenBiases == NULL ) m_pfHiddenBiases = new Real[m_unNumberOfHiddenNodes];
   for( UInt32 i = 0; i < m_unNumberOfHiddenNodes; i++ ) {
      m_pfHiddenBiases[i] = params[unChromosomePosition++]*(m_cBiasesBounds.GetMax() - m_cBiasesBounds.GetMin() ) + m_cBiasesBounds.GetMin();
   }
   
   // Hindden to Outputs
   if( m_pfHiddenToOutputWeights == NULL ) m_pfHiddenToOutputWeights = new Real[m_unNumberOfHiddenNodes * m_unNumberOfOutputs];
   for( UInt32 i = 0; i < m_unNumberOfHiddenNodes * m_unNumberOfOutputs; i++ ) {
      // m_pfHiddenToOutputWeights[i] = params[unChromosomePosition++];
      m_pfHiddenToOutputWeights[i] = params[unChromosomePosition++]*(m_cWeightsBounds.GetMax() - m_cWeightsBounds.GetMin() ) + m_cWeightsBounds.GetMin();
   }

   // Output Biases
   if( m_pfOutputBiases == NULL ) m_pfOutputBiases = new Real[m_unNumberOfOutputs];
   for( UInt32 i = 0; i < m_unNumberOfOutputs; i++ ) {
      m_pfOutputBiases[i] = params[unChromosomePosition++]*(m_cBiasesBounds.GetMax() - m_cBiasesBounds.GetMin() ) + m_cBiasesBounds.GetMin();
   }

   // Input to output weights
   if(m_pfInputToOutputWeights == NULL)   m_pfInputToOutputWeights = new Real[m_unNumberOfInputs * m_unNumberOfOutputs];
   for(UInt32 i = 0; i < m_unNumberOfInputs * m_unNumberOfOutputs;i++)
   {
      // m_pfInputToOutputWeights[i] = params[unChromosomePosition++];
      m_pfInputToOutputWeights[i] = params[unChromosomePosition++]*(m_cWeightsBounds.GetMax() - m_cWeightsBounds.GetMin() ) + m_cWeightsBounds.GetMin();
   }

   // Hidden
   if( m_pfHidden == NULL ) m_pfHidden = new Real[m_unNumberOfHiddenNodes];
   if(m_pfHiddenTime == NULL) m_pfHiddenTime = new Real[m_unNumberOfHiddenNodes];
   for( UInt32 i = 0; i < m_unNumberOfHiddenNodes; i++ ) {
      m_pfHidden[i]      = 0.0f;
      m_pfHiddenTime[i] = params[unChromosomePosition++];
   }
   // std::cout<<"unChromosomePosition"<<unChromosomePosition<<"\n";
}
