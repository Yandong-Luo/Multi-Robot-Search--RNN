#ifndef SINGLE_SPECIES_QTUSER_FUNCTION_H
#define SINGLE_SPECIES_QTUSER_FUNCTION_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CIRNNQTUserFunction : public CQTOpenGLUserFunctions {

public:

   CIRNNQTUserFunction();

   virtual ~CIRNNQTUserFunction() {}

   void Draw(CFootBotEntity& c_entity);
   
};

#endif
