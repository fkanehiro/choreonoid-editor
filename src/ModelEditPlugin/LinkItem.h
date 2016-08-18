/**
   \file
*/

#ifndef CNOID_EDITMODEL_PLUGIN_LINK_ITEM_H
#define CNOID_EDITMODEL_PLUGIN_LINK_ITEM_H

#include <cnoid/Item>
#include <cnoid/VRML>
#include <cnoid/SceneProvider>
#include "EditableModelBase.h"
#include "exportdecl.h"

namespace cnoid {
class Link;

class LinkItem;
typedef ref_ptr<LinkItem> LinkItemPtr;
class LinkItemImpl;

class CNOID_EXPORT LinkItem : public SceneProvider, public EditableModelBase
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    LinkItem();
    LinkItem(Link* link);
    LinkItem(const LinkItem& org);
    virtual ~LinkItem();

    bool loadModelFile(const std::string& filename);
    
    Link* link() const;
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
    friend class LinkItemImpl;
    LinkItemImpl* impl;
};
}

#endif
