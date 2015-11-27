/**
   \file
*/

#ifndef CNOID_EDITMODEL_PLUGIN_LINK_ITEM_H
#define CNOID_EDITMODEL_PLUGIN_LINK_ITEM_H

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/SceneProvider>
#include <cnoid/VRML>
#include <boost/optional.hpp>
#include "EditableModelBase.h"
#include "exportdecl.h"

namespace cnoid {

class LinkItem;
typedef ref_ptr<LinkItem> LinkItemPtr;
class LinkItemImpl;

class CNOID_EXPORT LinkItem : public Item, public SceneProvider, public EditableModelBase
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
    virtual ItemPtr doDuplicate() const;
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
