#include "rnn_qtuser_function.h"

#include <controllers/footbot_rnn/footbot_rnn_controller.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

// #include <string.h>

// #include <strstream>
/****************************************/
/****************************************/

CIRNNQTUserFunction::CIRNNQTUserFunction() {
   RegisterUserFunction<CIRNNQTUserFunction,CFootBotEntity>(&CIRNNQTUserFunction::Draw);
   // RegisterUserFunction<CDisasterQT,CFootBotEntity>(&CDisasterQT::Draw);
}

/****************************************/
/****************************************/

void CIRNNQTUserFunction::Draw(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   CFootBotRNNController& cController = dynamic_cast<CFootBotRNNController&>(c_entity.GetControllableEntity().GetController());

   UInt8 id = cController.GetRobotID();
   std::string str_id = std::to_string(id);
   Real fitness = cController.GetFitness();
   std::string str_fitness=std::to_string(fitness);
   str_id.append("f:");
   str_id.append(str_fitness);
   DrawText(CVector3(0.0, 0.0, 0.3),   // position
            str_id); // text

   //  /* Determine the type of robot */
   // // const std::string& name = CCI_Controller::GetId();
   // const std::string& str = c_entity.GetId().c_str();
   // std::string Target_label = "target";
   // std::string Nest_label  = "nest";
   // std::string Explorer_label = "explorer";
   // std::string::size_type t_label = str.find(Target_label);
   // std::string::size_type n_label = str.find(Nest_label);
   // std::string::size_type e_label = str.find(Explorer_label);


   // // const std::string& str = CCI_Controller::GetId();
   // // const std::string& str = c_entity.GetId().c_str();
   // std::string show = "ID:";
   // if (e_label != std::string::npos )     // Explorer
   // {
   //    std::string id = str.substr(8,str.length());
   //    CPhysarumMaze& cController = dynamic_cast<CPhysarumMaze&>(c_entity.GetControllableEntity().GetController());
   //    UInt8 type = cController.GetRobotType();
   //    UInt8 gradient = cController.GetNodeGradient();
   //    UInt8 root_id = cController.GetChainRoot();
   //    UInt8 C_node = cController.GetChildNode();
   //    UInt8 effect = cController.GetEffect();
   //    UInt8 type_nest = cController.GetTypeNest();
   //    std::string str_type=std::to_string(type);
   //    std::string str_root=std::to_string(root_id);
   //    std::string str_g=std::to_string(gradient);
   //    std::string str_c=std::to_string(C_node);
   //    std::string str_effect=std::to_string(effect);
   //    std::string str_type_nest = std::to_string(type_nest);
   //    show.append(id);
   //    show.append("T:");
   //    show.append(str_type);
   //    show.append("G:");
   //    show.append(str_g);
   //    show.append("R:");
   //    show.append(str_root);
   //    show.append("C:");
   //    show.append(str_c);
   //    show.append("E:");
   //    show.append(str_effect);
   //    show.append("NT:");
   //    show.append(str_type_nest);
   // }
   // else
   // {
   //    show = str;
      
   // }

   // DrawText(CVector3(0.0, 0.0, 0.3),   // position
   //          show); // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIRNNQTUserFunction, "rnn_qtuser_function")
