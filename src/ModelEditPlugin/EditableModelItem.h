/**
   \file
*/

#ifndef CNOID_EDITMODEL_PLUGIN_EDITABLEMODEL_ITEM_H
#define CNOID_EDITMODEL_PLUGIN_EDITABLEMODEL_ITEM_H

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/SceneProvider>
#include <boost/optional.hpp>
#include "exportdecl.h"

namespace cnoid {

class EditableModelItem;
typedef ref_ptr<EditableModelItem> EditableModelItemPtr;
class EditableModelItemImpl;

class CNOID_EXPORT EditableModelItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    EditableModelItem();
    EditableModelItem(const EditableModelItem& org);
    virtual ~EditableModelItem();

    bool loadModelFile(const std::string& filename);
    bool saveModelFile(const std::string& filename);
    
protected:
    virtual ItemPtr doDuplicate() const;
    virtual void doAssign(Item* item);
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    friend class EditableModelItemImpl;
    EditableModelItemImpl* impl;
};
}

#endif
