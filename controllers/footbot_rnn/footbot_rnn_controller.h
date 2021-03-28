#ifndef FOOTBOT_RNN_CONTROLLER
#define FOOTBOT_RNN_CONTROLLER

/*
 * Include some necessary headers.
 */
 #include <argos3/core/utility/math/rng.h>
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the colored_blob_omnidirectional_camera*/
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the foot-bot distance_scanner actuator */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_distance_scanner_actuator.h>
/* Definition of the foot-bot distance_scanner sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_distance_scanner_sensor.h>
/* Definition of the foot-bot range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the motor ground sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/*Definition of the RNN*/
#include "rnn.h"
// #include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/simulator/entity/positional_entity.h>
/* Definition of the perceptron */
// #include "nn/perceptron.h"

static const UInt32 m_unNumberOfInputProximity = 8;   // 8 proximity sensor was used
static const UInt32 m_unNumberOfInputGround = 1;
static const UInt32 m_unNumberOfInputCamera = 4;
static const UInt32 m_unNumberOfInputDS = 6;

const Real k = 400;

static Real energy_value;
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 * In this case, we also inherit from the CPerceptron class. We use
 * virtual inheritance so that matching methods in the CCI_Controller
 * and CPerceptron don't get messed up.
 */
class CFootBotRNNController : public CCI_Controller {

public:

   CFootBotRNNController();
   virtual ~CFootBotRNNController();

   void Init(TConfigurationNode& t_node);
   void ControlStep();
   void Reset();
   void Destroy();
   
   void Landmark_Init_Data();
   void Explorer_Init_Data();

   void Explorer_Motion();
   void Landmark_Motion();

   void Set_Blue_Side();
   void Set_Red_Side();

   void CalculateFitness();
   void GetInputFromCamera();
   void ShowInput(Real* inputs);

   void GetInputFromDistanceScanner();
   void GetInputFromProximity();

   void ShowInputFromDistanceScanner(Real* inputs);

   void GetInputFromGroundSensor();

   void determien_robot_type();

   void CalculateEnergy();
   void CalculateEnergyFitness();

   void RandomMove();
   CVector2 repulsion_vector();
   void Vector_to_Wheel_Velocity_Noscale(CVector2 direction);

   inline CRnn& GetCRNN() {
      return m_cRNN;
   }

   inline Real GetFitness() {
      return fitness_value;
   }

   inline UInt8 GetRobotID()
   {
      return ID;
   }

   inline Real GetEnterTimes()
   {
      return enter_times;
   }

   inline Real GetEnergyValue()
   {
      return energy_value;
   }

   inline Real GetPathState()
   {
      return exist_path;
   }

   void process_led();

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;
   /* Pointer to the foot-bot omnidirectional camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
   /*Pointer to the foot-bot LED sensor*/
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the foot-bot distance scanner actuator */
   CCI_FootBotDistanceScannerActuator* m_pcDSA;
   /* Pointer to the foot-bot distance scanner sensor */
   CCI_FootBotDistanceScannerSensor* m_pcDSS;
   /* Pointer to the foot-bot motor ground sensor */
   CCI_FootBotMotorGroundSensor* m_pcMGS;

   /* Pointer to the foot-bot range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the foot-bot range and bearing actuator */
   CCI_RangeAndBearingActuator* m_pcRABA;

   CPositionalEntity* centity;
   
   /* The recurrent neural network */
   CRnn m_cRNN;
   /* The perceptron neural network */
   // CPerceptron m_cPerceptron;
   /* Wheel speeds */
   Real m_fLeftSpeed, m_fRightSpeed;

   Real last_target_area;// initial is while

   Real fitness_value;

   UInt32 current_step;

   Real* m_pfInputDS;
   Real m_pfInputGround;
   Real* m_pfInputCamera;
   Real* m_pfInputProximity;

   UInt8 connet_landmark;     // 0 is not connect; 1 is connect

   Real energy_value;

   Real enter_times;

   Real exist_path;

   //ID
   UInt8 ID;

   UInt8 m_robot_type;

   // Real m_InputCamera[12];

   /* Current robot state */
   enum EState {
      STATE_LANDMARK = 0,
      STATE_EXPLORER = 1
   };
};

#endif
