#include "footbot_rnn_controller.h"

/****************************************/
/****************************************/

static CRange<Real> RNN_OUTPUT_RANGE(0.0f, 1.0f);
static CRange<Real> WHEEL_ACTUATION_RANGE(-10.0f, 10.0f);
// static CRange<Real> WHEEL_ACTUATION_RANGE(0.0f, 5.0f);


/****************************************/
/****************************************/

CFootBotRNNController::CFootBotRNNController():
   m_pcWheels(NULL),
   m_pcLight(NULL),
   m_pcProximity(NULL),
   m_pcCamera(NULL),
   m_pfInputCamera(NULL),
   m_pfInputDS(NULL),
   // m_pfInputGround(NULL),
   centity(NULL){}


/****************************************/
/****************************************/

CFootBotRNNController::~CFootBotRNNController() {
   // if(m_pcWheels) delete[] m_pcWheels;
   // if(m_pcLight)  delete[] m_pcLight;
   // if(m_pcProximity) delete[] m_pcProximity;
   // if(m_pcCamera) delete[] m_pcCamera;
   if(m_pfInputCamera) delete[] m_pfInputCamera;
   if(m_pfInputDS)   delete[] m_pfInputDS;
   // if(m_pfInputGround) delete[] m_pfInputGround;
}

/****************************************/
/****************************************/

void CFootBotRNNController::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    */
   try {
      m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
      m_pcRABS            = GetSensor  <CCI_RangeAndBearingSensor         >("range_and_bearing"       );
      m_pcRABA            = GetActuator<CCI_RangeAndBearingActuator       >("range_and_bearing"       );
      m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
      m_pcLight     = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );
      m_pcCamera    = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
      m_pcLEDs      = GetActuator<CCI_LEDsActuator                          >("leds");

      m_pcDSA       = GetActuator<CCI_FootBotDistanceScannerActuator>("footbot_distance_scanner");
      m_pcDSS       = GetSensor  <CCI_FootBotDistanceScannerSensor  >("footbot_distance_scanner");

      m_pcMGS       = GetSensor  <CCI_FootBotMotorGroundSensor      >("footbot_motor_ground"    );
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing sensors/actuators", ex);
   }

   // Enable camera filtering
   m_pcCamera->Enable();
   // Enable distance scanner
   m_pcDSA->Enable();
   m_pcDSA->SetRPM(300); // rounds per minutes


   // Initialize array pointer
   // Distance scanner
   m_pfInputDS = new Real[m_unNumberOfInputDS];
   ::memset(m_pfInputDS, 0, sizeof(Real) * m_unNumberOfInputDS);

   // Ground
   m_pfInputGround = 0;

   // camera
   m_pfInputCamera = new Real[m_unNumberOfInputCamera];
   ::memset(m_pfInputCamera,0,sizeof(Real)*m_unNumberOfInputCamera);

   //proximity
   m_pfInputProximity = new Real[m_unNumberOfInputProximity];
   ::memset(m_pfInputProximity,0,sizeof(Real)*m_unNumberOfInputProximity);
   
   determien_robot_type();

   last_target_area = 1;

   fitness_value = 0;

   current_step = 0;

   energy_value = 0;

   enter_times = 0;

   /* Initialize the perceptron */
   try {
      m_cRNN.Init(t_node);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the rnn network", ex);
   }
}


/****************************************/
/****************************************/

void CFootBotRNNController::ControlStep() {
   current_step++;
   // std::cout<<"current_step:"<<current_step<<"\n";
   switch(m_robot_type) {
      case STATE_EXPLORER:
         Explorer_Motion();
         break;
      case STATE_LANDMARK:
         Landmark_Motion();
         break;
      default:
         LOGERR << "[BUG] Unknown robot state: " << m_robot_type << std::endl;
   }
}

/****************************************/
/****************************************/
void CFootBotRNNController::Explorer_Motion()
{
   // GetInputFromDistanceScanner();
   // ShowInput(m_pfInputDS);
   // Clear all data
   m_pcRABA->SetData(0,0);
   if(current_step < 600)
   {
      RandomMove();
   }
   UInt32 temp_step = (current_step - 600);
   // if((current_step > 600 && (current_step-600)%3 == 0) || current_step == 600 )
   if(current_step > 600 || current_step == 600 )
   {
      // Get the Input from the different sensor
      GetInputFromDistanceScanner();
      GetInputFromGroundSensor();
      GetInputFromCamera();
      GetInputFromProximity();

      /* Fill RNN inputs from sensory data */
      for(size_t i=0;i<m_unNumberOfInputProximity;++i)
      {
         m_cRNN.SetInput(i,m_pfInputProximity[i]);
      }
      m_cRNN.SetInput(m_unNumberOfInputProximity, m_pfInputGround);
      for(size_t i = 0; i < m_unNumberOfInputCamera; ++i) {
         UInt32 index = 9+i;
         m_cRNN.SetInput(index, m_pfInputCamera[i]);
         // std::cout<<"index: "<<index<<" value:"<< m_pfInputCamera[i]<<"\n";
      }
      // m_cRNN.ShowInput();
      m_cRNN.ComputeOutputs();

      /*
      * Apply RNN outputs to actuation
      * The RNN outputs are in the range [0,1]
      * To allow for backtracking, we remap this range
      * into [-5:5] linearly.
      */
      RNN_OUTPUT_RANGE.MapValueIntoRange(
         m_fLeftSpeed,               // value to write
         m_cRNN.GetOutput(0), // value to read
         WHEEL_ACTUATION_RANGE       // target range (here [-5.0:5.0])
         );
      RNN_OUTPUT_RANGE.MapValueIntoRange(
         m_fRightSpeed,              // value to write
         m_cRNN.GetOutput(1), // value to read
         WHEEL_ACTUATION_RANGE       // target range (here [-5.0:5.0])
         );

      m_pcWheels->SetLinearVelocity(m_fLeftSpeed, m_fRightSpeed);
      if(m_cRNN.GetOutput(2) > 0.5)
      {
         Set_Blue_Side();
      }
      else
      {
         // shut down, we use the black to replace the situation of close the led
         m_pcLEDs->SetAllColors(CColor::BLACK);
      }
      if(m_cRNN.GetOutput(3) > 0.5)
      {
         Set_Red_Side();
      }
      else
      {
         m_pcLEDs->SetAllColors(CColor::BLACK);
      }
      // show output 
      // for(size_t i=0;i<4;i++)
      // {
      //    std::cout<<"output:"<<i<<" value:"<< m_cRNN.GetOutput(i)<<"\n";
      // }
      // CalculateFitness();
      CalculateEnergyFitness();
   }
}

// Activate blue led
void CFootBotRNNController::Set_Blue_Side()
{
   for(size_t i =0; i<12;i++)
   {
      if(i<3 || i>8)
      {
         m_pcLEDs->SetSingleColor(i, CColor::BLUE);
      }
   }
}

// Activate red led
void CFootBotRNNController::Set_Red_Side()
{
   for(size_t i =0; i<12;i++)
   {
      if(i>2 && i<9)
      {
         m_pcLEDs->SetSingleColor(i, CColor::RED);
      }
   }
}

void CFootBotRNNController::Landmark_Motion()
{
   m_pcRABA->SetData(0,0);
   for(size_t i =0; i<12;i++)
   {
     m_pcLEDs->SetSingleColor(i, CColor::RED);
   }
}


void CFootBotRNNController::CalculateFitness()
{
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcMGS->GetReadings();
   for(size_t i = 0; i < tGroundReads.size();i++)
   {
      if(tGroundReads[i].Value != last_target_area && tGroundReads[i].Value !=1)
      {
         last_target_area = tGroundReads[i].Value;
         fitness_value = fitness_value + 1;
         break;
      }
   }
}

void CFootBotRNNController::CalculateEnergyFitness()
{
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcMGS->GetReadings();
   bool enter_new_target_area = false;
   Real Energy_Consumption = 0.0;
   for(size_t i = 0; i < tGroundReads.size();i++)
   {
      if(tGroundReads[i].Value != last_target_area && tGroundReads[i].Value !=1)
      {
         last_target_area = tGroundReads[i].Value;
         fitness_value++;
         enter_new_target_area = true;
         break;
      }
   }

   // if(enter_new_target_area)
   // {
   //    if(enter_times == 1)
   //    {
   //       fitness_value = 0;
   //    }
   //    // 1 + Ed
   //    // ed = Energy_Consumption  * time step;
   //    // energy_value = 1 + (fabsf(WHEEL_ACTUATION_RANGE.GetMax()) + fabsf(WHEEL_ACTUATION_RANGE.GetMax()))/(2*k*WHEEL_ACTUATION_RANGE.GetMax())*134;
   //    // std::cout<<"first_enter_traget_energy:"<<energy_value<<"\n";
   //    fitness_value = fitness_value + energy_value;
   //    energy_value = 1 + 1/k *110/5.0 *10;
   //    // std::cout<<"enter_traget_energy:"<<energy_value<<"\n";
   //    Energy_Consumption = 0;
   // }
   // else
   // {
   //    // Energy consumption
   //    Energy_Consumption = (fabsf(m_fLeftSpeed) + fabsf(m_fRightSpeed))/(2*k*WHEEL_ACTUATION_RANGE.GetMax());
   //    // std::cout<<"consuption:"<<Energy_Consumption<<"m_fLeftSpeed:"<<m_fLeftSpeed<<"m_fRightSpeed:"<<m_fRightSpeed<<"\n";
   //    energy_value = energy_value - Energy_Consumption;
   // }
   // std::cout<<"Energy_Consumption:"<<Energy_Consumption<<"energy_value:"<<energy_value<<"fitness_value:"<< fitness_value<<"\n";
}

// void CFootBotRNNController::CalculateEnergyFitness()
// {
//    CalculateEnergy();
//    const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcMGS->GetReadings();
//    bool enter_new_target_area = false;
//    for(size_t i = 0; i < tGroundReads.size();i++)
//    {
//       if(tGroundReads[i].Value != last_target_area && tGroundReads[i].Value !=1)
//       {
//          last_target_area = tGroundReads[i].Value;
//          enter_new_target_area = true;
//          break;
//       }
//    }

//    // fmax = Vmax*All_time/ Distance (two different target)
//    // Real fmax =  WHEEL_ACTUATION_RANGE.GetMax()*540/110;
//    if(enter_new_target_area)
//    {
      
//       // fitness_value = fitness_value/fmax;
//    }
// }

/*The sensor value of the omnidirectional camera returns 1 if the robot detects the corresponding colored LED lights within the section, and 0 otherwise. 
*A total of twelve binary inputs are obtained through the omnidirectional camera: six inputs for detecting blue LED lights
* and another six for the red LED lights within the sensor range.
*/
void CFootBotRNNController::GetInputFromCamera()
{
   /* Get the camera readings */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   //
   ::memset(m_pfInputCamera,0,sizeof(Real)*m_unNumberOfInputCamera);
   /* Go through the camera readings to calculate the flocking interaction vector */
   if(! sReadings.BlobList.empty()) {
      for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {
         /*
          * The camera perceives the light as a yellow blob
          * The robots have their red beacon on
          * So, consider only red blobs
          * In addition: consider only the closest neighbors, to avoid
          * attraction to the farthest ones. Taking 180% of the target
          * distance is a good rule of thumb.
          */
         if(sReadings.BlobList[i]->Distance < 70)
         {
            Real angle  = sReadings.BlobList[i]->Angle.GetValue();
            // for(size_t j = 0; j < 4;++j)
            // {
            if(angle > 0 && angle < (2*CRadians::PI / 5).GetValue() )
            {
               if(sReadings.BlobList[i]->Color == CColor::BLUE)
               {
                  m_pfInputCamera[0] = 1;
               }
               else if(sReadings.BlobList[i]->Color == CColor::RED)
               {
                  m_pfInputCamera[1] = 1;
               }
            }
            else if(angle < 0 && angle > -(2*CRadians::PI / 5).GetValue() )
            {
               if(sReadings.BlobList[i]->Color == CColor::BLUE)
               {
                  m_pfInputCamera[2] = 1;
               }
               else if(sReadings.BlobList[i]->Color == CColor::RED)
               {
                  m_pfInputCamera[3] = 1;
               }
            }
         }
      }
   }
}

void CFootBotRNNController::ShowInput(Real* inputs)
{
   for(size_t i =0; i<sizeof(inputs);i++)
   {
      std::cout<<ID<<"index:"<<i<<"Input:"<< inputs[i]<<"\n";
   }
}

/*
Sensor values for the distance sensors are normalized as real values within the range of [0,1];
Returning 0 when no object is within the sensor range, and 1 if the robot collides with an object.*/
void CFootBotRNNController::GetInputFromDistanceScanner()
{
   // const CCI_FootBotDistanceScannerSensor::TReadingsMap& tReads = m_ds->GetShortReadingsMap();
   /* Get readings from distance scanner sensor */
   const CCI_FootBotDistanceScannerSensor::TReadingsMap& tDisranceReads = m_pcDSS->GetReadingsMap();
   ::memset(m_pfInputDS, 0, sizeof(Real) * m_unNumberOfInputDS);
   // CVector2 cAccumulator;
   for(CCI_FootBotDistanceScannerSensor::TReadingsMap::const_iterator it = tDisranceReads.begin(); it != tDisranceReads.end(); ++it)
   {	
      // printf("%f\t%f\t\n",it->second, it->first.GetValue());   
      // const_iterator
      for(size_t j=0;j<m_unNumberOfInputDS;j++)
      {
         Real angle = (-CRadians::PI / 2 + j*CRadians::PI / 6).GetValue();
         if(it->first.GetValue() == angle)
         {
            if(it->second < 2.5  && it->second != -2)
            {
               m_pfInputDS[j] = 1;
            }
         }
      }
   }
}

void CFootBotRNNController::GetInputFromProximity()
{
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

   ::memset(m_pfInputProximity, 0, sizeof(Real) * m_unNumberOfInputProximity);
   for(size_t i = 0; i < tProxReads.size(); i=i+3) {
      size_t j = i/3;
      m_pfInputProximity[j] = tProxReads[i].Value;
      // cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }

   // cAccumulator = -cAccumulator/8;     // tProxReads.size() = 24
}

/*The ground sensor is attached underneath the robot to detect whether the robot is inside or outside
 a target area: a binary value of 1 is returned if the robot is inside a target area, and 0 otherwise.*/
void CFootBotRNNController::GetInputFromGroundSensor()
{
   // The robot is out of the nest when all the motor ground
	// sensors see white (reading = 1)
   // bool flag = true;
   m_pfInputGround = 0;
   const CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_pcMGS->GetReadings();
   for(size_t i = 0; i < tGroundReads.size();i++)
   {
      if(tGroundReads[i].Value != 1)
      {
         // flag = false;
         m_pfInputGround = 1;
         break;
      }
   }
}

void CFootBotRNNController::ShowInputFromDistanceScanner(Real* inputs)
{
   for(size_t i =0; i<7;i++)
   {
      std::cout<<ID<<"index:"<<i<<"InputScanner:"<< inputs[i]<<"\n";
   }
}

void CFootBotRNNController::RandomMove()
{
   /*Calculate the advoid obstacles direction*/
   CVector2 advoid_direction = repulsion_vector();
   /*Generate a random vector*/

   // CVector2 random_direction = CVector2((Real)(rand()%10)/10,(Real)(rand()%10)/10);
   // CVector2 sum_direction = advoid_direction+random_direction;

   /*Moving according to the direction*/
   // Vector_to_Wheel_Velocity_Noscale(advoid_direction);

   Real temp = advoid_direction.GetX()*advoid_direction.GetX()+advoid_direction.GetY()*advoid_direction.GetY();
   
   if ( temp > 0.001)
   {
      Vector_to_Wheel_Velocity_Noscale(advoid_direction);
   }
   else
   {
      m_pcWheels->SetLinearVelocity(5.0,5.0);
   }
}


CVector2 CFootBotRNNController::repulsion_vector()
{
   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

   /* Sum them together */
   /* length*cos(angle),length*sin(angle)*/
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); i=i+3) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }

   cAccumulator = -cAccumulator/8;     // tProxReads.size() = 24
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   
   // CRadians cAngle = cAccumulator.Angle();
   // if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) && cAccumulator.Length() < m_fDelta) {
   //    cAccumulator = {0,0};
   // }
   // std::cout << "vector:" << cAccumulator << "\n";
   return cAccumulator;
}

/* Transforms the given direction vector into wheel actuation
  This version does not consider the length of the direction vector*/
void CFootBotRNNController::Vector_to_Wheel_Velocity_Noscale(CVector2 direction)
{
   Real m_fWheelVelocity = 5.0;
    // Get the direction angle in (-pi,pi)
   // Real angle = direction.Angle();
   Real angle = atan2(direction.GetY(),direction.GetX());
    // Declare speeds
   Real lvel = 0;
   Real rvel = 0;
   if (angle > 0)
   {
      lvel = m_fWheelVelocity * (1 - 2 * angle  / M_PI);
      rvel = m_fWheelVelocity;
   }
   else
   {
      lvel = m_fWheelVelocity;
      rvel = m_fWheelVelocity * (1 + 2 * angle  / M_PI);
   }
    // Set wheel speeds
   //  std::cout<<"left_wheel:"<<lvel<<"right_wheel:"<<rvel<<"\n";
    m_pcWheels->SetLinearVelocity(lvel, rvel);
   //  m_pcWheels->SetLinearVelocity(m_fWheelVelocity
}

/****************************************/
/****************************************/

void CFootBotRNNController::determien_robot_type() {

   /* Determine the type of robot */
   const std::string& name = CCI_Controller::GetId();
   std::string landmark_label = "landmark";
   std::string explorer_label  = "explorer";
   std::string::size_type l_label = name.find(landmark_label);
   std::string::size_type e_label = name.find(explorer_label);


   const std::string& str = CCI_Controller::GetId();
   if(l_label != std::string::npos)          // is landmark
   {
      Landmark_Init_Data();
   }
   else if(e_label != std::string::npos)     // is explorer
   {
      Explorer_Init_Data();
   }
}

void CFootBotRNNController::Reset()
{
   m_cRNN.Reset();
   // reset the distance scanner vector
   ::memset(m_pfInputDS, 0, sizeof(Real) * m_unNumberOfInputDS);
   // reset the ground vector
   // ::memset(m_pfInputGround,0,sizeof(Real)*m_unNumberOfInputGround);
   m_pfInputGround = 0;
   // reset the camera vector
   ::memset(m_pfInputCamera,0,sizeof(Real)*m_unNumberOfInputCamera);

   determien_robot_type();

   current_step = 0;

   last_target_area = 1;

   fitness_value = 0;

   energy_value = 0;

   enter_times = 0;
}

void CFootBotRNNController::Landmark_Init_Data()
{
   /*type*/
   m_robot_type = STATE_LANDMARK;

   /*ID*/
   const std::string& str = CCI_Controller::GetId();
   std::string target_id = str.substr(8,str.length());
   ID = atoi(target_id.c_str());

   // Clear all data
   m_pcRABA->ClearData();

   m_pcRABA->SetData(0,m_robot_type);

   m_pcRABA->SetData(1,ID);
}


void CFootBotRNNController::Explorer_Init_Data()
{
   /*type*/
   m_robot_type = STATE_EXPLORER;

   /*ID*/
   const std::string& str = CCI_Controller::GetId();
   std::string target_id = str.substr(8,str.length());
   ID = atoi(target_id.c_str());

   // Clear all data
   m_pcRABA->ClearData();

   m_pcRABA->SetData(0,m_robot_type);
}


/****************************************/
/****************************************/

void CFootBotRNNController::Destroy() {
   m_cRNN.Destroy();
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotRNNController, "footbot_rnn_controller")
