/**
   \file
*/

#ifndef CNOID_EDITMODEL_PLUGIN_EDITABLEMODELBASE_ITEM_H
#define CNOID_EDITMODEL_PLUGIN_EDITABLEMODELBASE_ITEM_H

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/SceneProvider>
#include <cnoid/VRML>
#include <cnoid/VRMLBodyLoader>
#include <boost/optional.hpp>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT EditableModelBase
{
public:
    EditableModelBase();
    VRMLNodePtr originalNode;
    Vector3 translation;
    Matrix3 rotation;
    virtual VRMLNodePtr toVRML() {};
    bool onTranslationChanged(const std::string& value);
    bool onRotationChanged(const std::string& value);
    bool onRotationAxisChanged(const std::string& value);
    bool onRotationRPYChanged(const std::string& value);
    void doPutProperties(PutPropertyFunction& putProperty);
};

}

#endif
