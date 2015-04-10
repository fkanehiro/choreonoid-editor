#include "EditableModelItem.h"
#include "LinkItem.h"
#include "PrimitiveShapeItem.h"
#include "JointItem.h"
#include "SensorItem.h"
#include <cnoid/Plugin>

using namespace cnoid;

class ModelEditPlugin : public Plugin
{
public:
    
    ModelEditPlugin() : Plugin("ModelEdit") { }
    
    virtual bool initialize() {
        
        EditableModelItem::initializeClass(this);
        LinkItem::initializeClass(this);
        PrimitiveShapeItem::initializeClass(this);
        JointItem::initializeClass(this);
        SensorItem::initializeClass(this);
        
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(ModelEditPlugin);
