/**
   \file
*/

#ifndef CNOID_EDITMODEL_PLUGIN_MESHSHAPE_ITEM_H
#define CNOID_EDITMODEL_PLUGIN_MESHSHAPE_ITEM_H

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

class MeshShapeItem;
typedef ref_ptr<MeshShapeItem> MeshShapeItemPtr;
class MeshShapeItemImpl;

class CNOID_EXPORT MeshShapeItem : public SceneProvider, public EditableModelBase
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    MeshShapeItem();
    MeshShapeItem(const MeshShapeItem& org);
    virtual ~MeshShapeItem();

    VRMLNodePtr toVRML();
    std::string toURDF();

    virtual SgNode* getScene();

protected:
    virtual Item* doDuplicate() const;
    virtual void doAssign(Item* item);
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    friend class MeshShapeItemImpl;
    MeshShapeItemImpl* impl;
};
}

#endif
