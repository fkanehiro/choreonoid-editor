/**
   \file
*/

#ifndef CNOID_EDITMODEL_PLUGIN_PRIMITIVESHAPE_ITEM_H
#define CNOID_EDITMODEL_PLUGIN_PRIMITIVESHAPE_ITEM_H

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/SceneProvider>
#include <cnoid/VRML>
#include <cnoid/VRMLBodyLoader>
#include <boost/optional.hpp>
#include "EditableModelBase.h"
#include "exportdecl.h"

namespace cnoid {

class PrimitiveShapeItem;
typedef ref_ptr<PrimitiveShapeItem> PrimitiveShapeItemPtr;
class PrimitiveShapeItemImpl;

class CNOID_EXPORT PrimitiveShapeItem : public Item, public SceneProvider, public EditableModelBase
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    PrimitiveShapeItem();
    PrimitiveShapeItem(Link* link);
    PrimitiveShapeItem(const PrimitiveShapeItem& org);
    virtual ~PrimitiveShapeItem();

    Link* link() const;
    VRMLNodePtr toVRML();

    virtual SgNode* getScene();

protected:
    virtual ItemPtr doDuplicate() const;
    virtual void doAssign(Item* item);
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    friend class PrimitiveShapeItemImpl;
    PrimitiveShapeItemImpl* impl;
};
}

#endif
