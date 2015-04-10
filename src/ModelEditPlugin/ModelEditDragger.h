
/**
*/

#ifndef CNOID_MODELEDIT_DRAGGER_H
#define CNOID_MODELEDIT_DRAGGER_H

#include <cnoid/SceneDragger>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ModelEditDragger : public PositionDragger
{
public:
    // override scene mode change event to show always
    virtual void onSceneModeChanged(const SceneWidgetEvent& event) {};
};

typedef ref_ptr<ModelEditDragger> ModelEditDraggerPtr;
}

#endif
