#ifndef RNN_H
#define RNN_H

#include "neural_network.h"
#include <argos3/core/utility/math/range.h>

class CRnn : public CNeuralNetwork {

public:

   CRnn();
   virtual ~CRnn();

   virtual void Init(TConfigurationNode& t_tree);
   // virtual void Reset();
   virtual void Destroy();

   virtual void LoadNetworkParameters(const std::string& str_filename);
   virtual void LoadNetworkParameters(const UInt32 un_num_params,
                                      const Real* pf_params );
   virtual void ComputeOutputs();

   void showWeightParameters(const UInt32 un_num_params,const Real* params);


   inline  UInt32      GetNumberOfHiddenNodes() { return m_unNumberOfHiddenNodes; }
   inline  const Real* GetHidden()              { return m_pfHidden;        }
//    inline  const Real* GetHiddenTaus()          { return m_pfHiddenTaus;          }
//    inline  const Real* GetHiddenBias()          { return m_pfHiddenBiases;        }
//    inline  const Real* GetOutputBias()          { return m_pfOutputBiases;        }

protected:

   Real* m_pfInputToHiddenWeights;

   Real* m_pfHiddenToHiddenWeights;

   Real* m_pfHiddenToOutputWeights;
   Real* m_pfOutputBiases;

   Real* m_pfInputToOutputWeights;

   Real* m_pfHiddenBiases;
   Real* m_pfHidden;

   Real* m_pfHiddenTime;

   UInt32 m_unNumberOfHiddenNodes;

   UInt32 m_unNumberOfWeightsFromInputToWeights;

   CRange<Real> m_cWeightsBounds;
   // CVector2
   CRange<Real> m_cBiasesBounds;
   CRange<Real> m_cTimeBounds;

//    CRange<Real> m_cTausBounds;

};

#endif

