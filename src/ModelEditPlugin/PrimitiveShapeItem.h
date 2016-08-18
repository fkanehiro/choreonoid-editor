/**
   \file
*/

#ifndef CNOID_EDITMODEL_PLUGIN_PRIMITIVESHAPE_ITEM_H
#define CNOID_EDITMODEL_PLUGIN_PRIMITIVESHAPE_ITEM_H

#include <cnoid/Item>
#include <cnoid/VRML>
#include "EditableModelBase.h"
#include "exportdecl.h"

namespace cnoid {
class SgShape;

class PrimitiveShapeItem;
typedef ref_ptr<PrimitiveShapeItem> PrimitiveShapeItemPtr;
class PrimitiveShapeItemImpl;

class CNOID_EXPORT PrimitiveShapeItem : public SceneProvider, public EditableModelBase
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    PrimitiveShapeItem();
    PrimitiveShapeItem(const Vector3& translation, const Matrix3& rotation,
                       SgShape *shape);
    PrimitiveShapeItem(const PrimitiveShapeItem& org);
    virtual ~PrimitiveShapeItem();

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
    friend class PrimitiveShapeItemImpl;
    PrimitiveShapeItemImpl* impl;
};
}

#endif
