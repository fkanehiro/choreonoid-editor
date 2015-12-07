/**
   \file
*/

#ifndef CNOID_EDITMODEL_PLUGIN_JOINT_ITEM_H
#define CNOID_EDITMODEL_PLUGIN_JOINT_ITEM_H

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/SceneProvider>
#include <cnoid/VRML>
#include <boost/optional.hpp>
#include "EditableModelBase.h"
#include "exportdecl.h"

namespace cnoid {

class JointItem;
typedef ref_ptr<JointItem> JointItemPtr;
class JointItemImpl;

class CNOID_EXPORT JointItem : public Item, public SceneProvider, public EditableModelBase
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    JointItem();
    JointItem(const JointItem& org);
    JointItem(Link* link);
    virtual ~JointItem();

    VRMLNodePtr toVRML();
    std::string toURDF();
    
    Link* link() const;
    
    virtual SgNode* getScene();

protected:
    virtual Item* doDuplicate() const;
    virtual void doAssign(Item* item);
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    friend class JointItemImpl;
    JointItemImpl* impl;
};
}

#endif
