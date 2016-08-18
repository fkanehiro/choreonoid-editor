/**
   \file
*/

#ifndef CNOID_EDITMODEL_PLUGIN_EDITABLEMODELBASE_ITEM_H
#define CNOID_EDITMODEL_PLUGIN_EDITABLEMODELBASE_ITEM_H

#include <cnoid/Item>
#include <cnoid/VRML>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

std::vector<double> readvector(const std::string& value);

class CNOID_EXPORT EditableModelBase : public Item
{
public:
    EditableModelBase();
    VRMLNodePtr originalNode;
    Vector3 translation, absTranslation;
    Matrix3 rotation, absRotation;
    virtual VRMLNodePtr toVRML() {};
    virtual std::string toURDF() {};
    bool onTranslationChanged(const std::string& value);
    bool onRotationChanged(const std::string& value);
    bool onRotationAxisChanged(const std::string& value);
    bool onRotationRPYChanged(const std::string& value);
    void doPutProperties(PutPropertyFunction& putProperty);
    void updateChildPositions();
    void updatePosition();
};

}

#endif
