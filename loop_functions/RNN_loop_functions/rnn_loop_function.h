#ifndef RNN_LOOP_FUNCTIONS_H
#define RNN_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <argos3/core/utility/math/rng.h>

/* The RNN controller */
#include <controllers/footbot_rnn/footbot_rnn_controller.h>

/* GA-related headers */
#include <ga/ga.h>
#include <ga/GARealGenome.h>
#include <ga/GARealGenome.C> // this is necessary!

static const size_t GENOME_SIZE = 113;

const UInt8 group_size = 15;

using namespace argos;

class CRNNLoopFunction : public CLoopFunctions {

public:

   CRNNLoopFunction();
   virtual ~CRNNLoopFunction();

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PostExperiment();
   virtual bool IsExperimentFinished();
   // virtual void Destroy();

   /* Called by the evolutionary algorithm to set the current trial */
   inline void SetTrial(size_t un_trial) {
      m_unCurrentTrial = un_trial;
   }

   // inline CSimulator& GetSimulator() const {
   //       return *m_pcSimulator;
   //    }

   /* Configures the robot controller from the genome */
   void ConfigureFromGenome(const GARealGenome& c_genome);

   void set_robot();

   /* Calculates the performance of the robot in a trial */
   Real Performance();

private:
   
   size_t m_unCurrentTrial;
   CFootBotEntity* c_entity;
   CFootBotRNNController* cController;
   Real* m_pfControllerParams;
   CRange<Real> m_cArenaSideX, m_cArenaSideY;

   CVector2 m_CircleCenter1;
   CVector2 m_CircleCenter2;
   Real m_CircleRadius1;
   Real m_CircleRadius2;
   CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
   CFloorEntity* m_pcFloor;

   /* The initial setup of a trial */
   struct SInitSetup {
      CVector3 Position;
      CQuaternion Orientation;
   };

   
   std::vector<SInitSetup> m_vecInitSetup;

   CVector3 random_position;
   CQuaternion random_orientation;

   CRandom::CRNG* m_pcRNG;

   std::string m_strOutput;
   std::ofstream m_cOutput;
   std::string m_TimeOutput;
   std::ofstream m_tOutput;
   std::string m_SROutput;
   std::ofstream m_srOutput;

   std::vector<CFootBotEntity*> m_vecTargets;

   UInt32 m_PathForm;

   UInt32 FinishedTime;

   bool allFound;
};

#endif
