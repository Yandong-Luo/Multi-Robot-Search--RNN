#include "rnn_loop_function.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <math.h>
// #include <controllers/single_species_without_floor/physarum_maze.h>
// #include <controllers/single_species_without_floor/footbot_nest_and_target.h>

CRNNLoopFunction::CRNNLoopFunction():
   m_cArenaSideX(-2.4f, 2.4f),
   m_cArenaSideY(-2.4f, 2.4f),
   m_unCurrentTrial(0),
   m_vecInitSetup(5),
   c_entity(NULL),
   cController(NULL),
   m_pfControllerParams(new Real[GENOME_SIZE]),
   m_pcRNG(NULL) {}

CRNNLoopFunction::~CRNNLoopFunction() {
   delete[] m_pfControllerParams;
}

void CRNNLoopFunction::Init(TConfigurationNode& t_node) {
   /* Parse parameters */
   GetNodeAttribute(t_node, "circle_center_1", m_CircleCenter1);
   GetNodeAttribute(t_node, "circle_radius_1", m_CircleRadius1);
   GetNodeAttribute(t_node, "circle_center_2", m_CircleCenter2);
   GetNodeAttribute(t_node, "circle_radius_2", m_CircleRadius2);
   m_pcFloor = &GetSpace().GetFloorEntity();

   set_robot();

   /*
    * Process trial information, if any
    */
   try {
      GetNodeAttribute(t_node, "trial", m_unCurrentTrial);


      // TConfigurationNode& total_output = GetNode(t_node, "total_output_file");
      TConfigurationNode& time_output  = GetNode(t_node,"time_output_file");
      TConfigurationNode& success_rate_output = GetNode(t_node,"success_rate_output_file");

      // /* Get the output file name from XML */
      // GetNodeAttribute(total_output, "output", m_strOutput);
      GetNodeAttribute(time_output, "output", m_TimeOutput);
      GetNodeAttribute(success_rate_output, "output", m_SROutput);
      // /* Open the file, erasing its contents */
      // m_cOutput.open(m_strOutput.c_str(), std::ios_base::ate  | std::ios_base::app);
      m_tOutput.open(m_TimeOutput.c_str(), std::ios_base::ate  | std::ios_base::app);
      m_srOutput.open(m_SROutput.c_str(), std::ios_base::ate  | std::ios_base::app);

      // /* Get handle to the landmark */
      CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
      for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();it != m_cFootbots.end();++it) {
         CFootBotEntity* cFootBot = any_cast<CFootBotEntity*>(it->second);
         // std::cout<< "0-6:"<< cFootBot->GetId().substr(0, 6) <<"\n";
         if(cFootBot->GetId().length() >=8 && cFootBot->GetId().substr(0, 8) == "explorer") {
            m_vecTargets.push_back(cFootBot);
         }
      }

      Reset();
   }
   catch(CARGoSException& ex) {}
}

void CRNNLoopFunction::set_robot()
{
   /*
    * Create the random number generator
    */
   m_pcRNG = CRandom::CreateRNG("argos");

   /*
    * Create the foot-bot and get a reference to its controller
    */
   for(size_t i = 0; i<group_size;i++)
   {
      // ID
      std::string str_id = "explorer";
      str_id.append(std::to_string(i+1));

      c_entity = new CFootBotEntity(str_id,"frnn");

      AddEntity(*c_entity);
   }
}


/****************************************/
/****************************************/

void CRNNLoopFunction::Reset() {
   // m_PathForm = 0;
   // FinishedTime = 0;

   /*
    * Create the random number generator
    */
   m_pcRNG = CRandom::CreateRNG("argos");

   // size_t num_success_adden = 0;
   
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();it != m_cFootbots.end();++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
      
      // Get the id
      const std::string& str = cFootBot.GetId().c_str();
      std::string Explorer_label = "explorer";
      std::string::size_type e_label = str.find(Explorer_label);

      if (e_label != std::string::npos )     // explorer
      {
         random_position.Set(m_pcRNG->Uniform(m_cArenaSideX),m_pcRNG->Uniform(m_cArenaSideY),0);
         CRadians cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
         random_orientation.FromEulerAngles(
            cOrient,        // rotation around Z
            CRadians::ZERO, // rotation around Y
            CRadians::ZERO  // rotation around X
            );
         bool success_add = false;
         do{
            if(MoveEntity(
               cFootBot.GetEmbodiedEntity(),             // move the body of the robot
               random_position,    // to this position
               random_orientation, // with this orientation
               false                                         // this is not a check, leave the robot there
               )) {
                  success_add = true;
                  // num_success_adden++;
                  // std::cout<<"has adden explorer:"<<num_success_adden<<"\n";
            }
            else
            {
               success_add = false;
               // LOGERR << "Can't move explorer:"<< num_success_adden<<" robot in <"
               //     << random_position
               //     << ">, <"
               //     << random_orientation
               //     << ">"
               //     << std::endl;
               random_position.Set(m_pcRNG->Uniform(m_cArenaSideX),m_pcRNG->Uniform(m_cArenaSideY),0);
               CRadians cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
               random_orientation.FromEulerAngles(
               cOrient,        // rotation around Z
               CRadians::ZERO, // rotation around Y
               CRadians::ZERO  // rotation around X
               );
            }
         }while(success_add == false);
         
      }
      else{
         std::string id = str.substr(8,str.length());

         int int_id=atoi(id.c_str());

         
         if(int_id == 1)
         {
            random_position.Set(1.0,0,0);
            random_orientation.FromEulerAngles(
            CRadians::ZERO, // rotation around Z
            CRadians::ZERO, // rotation around Y
            CRadians::ZERO  // rotation around X
            );
         }
         else
         {
            random_position.Set(-0.8,0,0);
            random_orientation.FromEulerAngles(
            CRadians::ZERO, // rotation around Z
            CRadians::ZERO, // rotation around Y
            CRadians::ZERO  // rotation around X
            );
         }
         // while(size_t j<)
         if(!MoveEntity(
               cFootBot.GetEmbodiedEntity(),             // move the body of the robot
               random_position,    // to this position
               random_orientation, // with this orientation
               false                                         // this is not a check, leave the robot there
               )) {
            LOGERR << "Can't move robot in <"
                   << random_position
                   << ">, <"
                   << random_orientation
                   << ">"
                   << std::endl;
            }
      }
   }
}

/****************************************/
/****************************************/

CColor CRNNLoopFunction::GetFloorColor(const CVector2& c_position_on_plane) {
   Real circle1 = pow((c_position_on_plane.GetX()-m_CircleCenter1.GetX()),2) + pow((c_position_on_plane.GetY()-m_CircleCenter1.GetY()),2);
   Real pow_circle1_radius = pow(m_CircleRadius1,2);
   Real circle2 = pow((c_position_on_plane.GetX()-m_CircleCenter2.GetX()),2) + pow((c_position_on_plane.GetY()-m_CircleCenter2.GetY()),2);
   Real pow_circle2_radius = pow(m_CircleRadius2,2);
   // std::pow((c_position_on_plane.GetX()-m_CircleCenter1.GetX()),2)
   if(circle1 < pow_circle1_radius)
   {
      return CColor::GRAY50;
   }
   if(circle2 < pow_circle2_radius)
   {
      return CColor::GRAY60;
   }
   return CColor::WHITE;
}



/****************************************/
/****************************************/

void CRNNLoopFunction::ConfigureFromGenome(const GARealGenome& c_genome) {
   /* Copy the genes into the NN parameter buffer */
   for(size_t i = 0; i < GENOME_SIZE; ++i) {
      m_pfControllerParams[i] = c_genome[i];
   }
   
   /* Set the NN parameters */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();it != m_cFootbots.end();++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);

      // Get the id
      const std::string& str = cFootBot.GetId().c_str();
      std::string Explorer_label = "explorer";
      std::string::size_type e_label = str.find(Explorer_label);

      if (e_label != std::string::npos )     // explorer
      {
         cController = &dynamic_cast<CFootBotRNNController&>(cFootBot.GetControllableEntity().GetController());
         cController->GetCRNN().SetOnlineParameters(GENOME_SIZE, m_pfControllerParams);
      }
   }
}

Real CRNNLoopFunction::Performance() {
   Real sum_fitness = 0.0;
   // std::cout<<"sum_fitness:"<<sum_fitness<<"\n";
   /* Check whether a robot is on a food item */
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
   for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();it != m_cFootbots.end();++it) {
      /* Get handle to foot-bot entity and controller */
      CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);

      // Get the id
      const std::string& str = cFootBot.GetId().c_str();
      std::string Explorer_label = "explorer";
      std::string::size_type e_label = str.find(Explorer_label);

      // fmax = Vmax*All_time/ Distance (two different target)
      Real fmax =  8.2*540/110;
      // std::cout<<"fmax:"<< fmax<<"\n";
      if (e_label != std::string::npos )     // explorer
      {
         cController = &dynamic_cast<CFootBotRNNController&>(cFootBot.GetControllableEntity().GetController());

         // Get the fitness
         Real fitness = cController->GetFitness();

         // Real enter_times = cController->GetEnterTimes();

         // Real energy_value = cController->GetEnergyValue();

         UInt8 id = cController->GetRobotID();

         Real Fi = fitness;

         sum_fitness = sum_fitness + Fi;

         std::cout<<"\n"<<"robot:"<< id<<" enter_times:"<< fitness << " sum_fitness:"<<sum_fitness<<"\n";
      }
   }
   // if(sum_fitness != 0)
   // {
   sum_fitness = sum_fitness/group_size;
   // sum_fitness = -sum_fitness;
   // }
   // else
   // {
   //    sum_fitness = 200*group_size;
   // }
   std::cout<<"final_fitness:"<<sum_fitness<<"\n";
   // Real result = 
   /* The performance is simply the distance of the robot to the origin */
   return sum_fitness;
}

bool CRNNLoopFunction::IsExperimentFinished()
{
   bool IsFinished = false;

   UInt32 max_clock = GetSimulator().GetMaxSimulationClock();
   UInt32 cur_clock = GetSpace().GetSimulationClock();

   FinishedTime = 0;

   // std::cout<<"MaxSimulationClock:"<<max_clock<<"Current_clock:"<<cur_clock<<"\n";
   /* Reset counter */

   Real all_enter_times = 0;
   /* Go through victims */
   for(size_t i = 0; i < m_vecTargets.size(); ++i) {
      /* Get reference to robot controller */
      CFootBotRNNController& cContr = dynamic_cast<CFootBotRNNController&>(m_vecTargets[i]->GetControllableEntity().GetController());
      Real m_enter_times = cContr.GetFitness();
      all_enter_times = all_enter_times + m_enter_times;
   }
   Real aver_enter_times = all_enter_times/m_vecTargets.size();
   // std::cout<<"aver_times:"<<aver_enter_times<<"\n";
   /* Check if the simulation must be terminated */
   if(aver_enter_times >= 5)
   {
      // std::cout<<m_vecTargets.size()<<"aaa\n";
      allFound = true;
      IsFinished = true;
      FinishedTime = cur_clock;
      /* Output stuff to file */
      // m_cOutput << cur_clock << "\t";
   }
   else if(max_clock > 0 && cur_clock >= max_clock)   /* Check simulation clock */
   {
      allFound = false;
      IsFinished = true;
      FinishedTime = max_clock;
   }
   return IsFinished;
}

void CRNNLoopFunction::PostExperiment()
{
   // std::cout<<"postExp" << "\n";

   /* Output stuff to file */
   // m_cOutput << GetSpace().GetSimulationClock() << "\t"
   //           << finish << "\t"
   //           << std::endl;
   m_tOutput << FinishedTime <<" ";
   // m_cOutput<<m_TargetsFound<<std::endl;
   if(allFound)
   {
      m_srOutput<<allFound;
   }
   // m_srOutput << allFound <<" "<< std::endl;
   std::cout<<"FINISHED_TIME:"<< FinishedTime<<" all_Found:" << allFound << "\n";
}

void CRNNLoopFunction::Destroy() {
   /* Close the file */
   // m_cOutput.close();
   m_tOutput.close();
   m_srOutput.close();
}

REGISTER_LOOP_FUNCTIONS(CRNNLoopFunction, "rnn_loop_function")
