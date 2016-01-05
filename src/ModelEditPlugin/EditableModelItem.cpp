/**
   @file
*/

#include "EditableModelItem.h"
#include "EditableModelBase.h"
#include "JointItem.h"
#include "LinkItem.h"
#include "SensorItem.h"
#include <cnoid/YAMLReader>
#include <cnoid/EigenArchive>
#include <cnoid/Archive>
#include <cnoid/RootItem>
#include <cnoid/LazySignal>
#include <cnoid/LazyCaller>
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/ItemTreeView>
#include <cnoid/OptionManager>
#include <cnoid/MenuManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/JointPath>
#include <cnoid/BodyLoader>
#include <cnoid/VRMLBodyLoader>
#include <cnoid/BodyState>
#include <cnoid/SceneBody>
#include "ModelEditDragger.h"
#include <cnoid/VRML>
#include <cnoid/VRMLBody>
#include <cnoid/VRMLBodyWriter>
#include <cnoid/FileUtil>
#include <sdf/sdf.hh>
#include <sdf/parser_urdf.hh>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include <bitset>
#include <deque>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = boost::filesystem;

namespace {

const bool TRACE_FUNCTIONS = false;

BodyLoader bodyLoader;

inline double radian(double deg) { return (3.14159265358979 * deg / 180.0); }

bool loadEditableModelItem(EditableModelItem* item, const std::string& filename)
{
    if(item->loadModelFile(filename)){
        return true;
    }
    return false;
}


bool saveEditableModelItem(EditableModelItem* item, const std::string& filename)
{
    if(item->saveModelFile(filename)){
        return true;
    }
    return false;
}
    
bool saveEditableModelItemURDF(EditableModelItem* item, const std::string& filename)
{
    if(item->saveModelFileURDF(filename)){
        return true;
    }
    return false;
}

bool saveEditableModelItemSDF(EditableModelItem* item, const std::string& filename)
{
    if(item->saveModelFileSDF(filename)){
        return true;
    }
    return false;
}

}


namespace cnoid {

class EditableModelItemImpl
{
public:
    EditableModelItem* self;

    EditableModelItemImpl(EditableModelItem* self);
    EditableModelItemImpl(EditableModelItem* self, const EditableModelItemImpl& org);
    ~EditableModelItemImpl();
    
    bool loadModelFile(const std::string& filename);
    bool saveModelFile(const std::string& filename);
    bool saveModelFileURDF(const std::string& filename);
    bool saveModelFileSDF(const std::string& filename);
    VRMLNodePtr toVRML();
    string toURDF();
    void setLinkTree(Link* link, VRMLBodyLoader* vloader);
    void setLinkTreeSub(Link* link, VRMLBodyLoader* vloader, Item* parentItem);
    void doAssign(Item* srcItem);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}


void EditableModelItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        ext->itemManager().registerClass<EditableModelItem>(N_("EditableModelItem"));
        ext->itemManager().addCreationPanel<EditableModelItem>();
        ext->itemManager().addLoader<EditableModelItem>(
            _("OpenHRP Model File for Editing"), "OpenHRP-VRML-MODEL", "wrl;dae;stl", boost::bind(loadEditableModelItem, _1, _2));
        ext->itemManager().addSaver<EditableModelItem>(
            _("OpenHRP Model File"), "OpenHRP-VRML-MODEL", "wrl", boost::bind(saveEditableModelItem, _1, _2));
        ext->itemManager().addSaver<EditableModelItem>(
            _("URDF Model File"), "URDF-MODEL", "urdf", boost::bind(saveEditableModelItemURDF, _1, _2));
        ext->itemManager().addSaver<EditableModelItem>(
            _("SDF Model File"), "SDF-MODEL", "sdf", boost::bind(saveEditableModelItemSDF, _1, _2));
        initialized = true;
    }
}


EditableModelItem::EditableModelItem()
{
    impl = new EditableModelItemImpl(this);
}


EditableModelItemImpl::EditableModelItemImpl(EditableModelItem* self)
    : self(self)
{
}


EditableModelItem::EditableModelItem(const EditableModelItem& org)
    : Item(org)
{
    impl = new EditableModelItemImpl(this, *org.impl);
}


EditableModelItemImpl::EditableModelItemImpl(EditableModelItem* self, const EditableModelItemImpl& org)
    : self(self)
{
}


EditableModelItem::~EditableModelItem()
{
    delete impl;
}


EditableModelItemImpl::~EditableModelItemImpl()
{
}


void EditableModelItemImpl::setLinkTree(Link* link, VRMLBodyLoader* vloader)
{
    setLinkTreeSub(link, vloader, self);
    self->notifyUpdate();
}


void EditableModelItemImpl::setLinkTreeSub(Link* link, VRMLBodyLoader* vloader, Item* parentItem)
{
    // first, create joint item
    JointItemPtr item = new JointItem(link);
    item->originalNode = vloader->getOriginalNode(link);
    parentItem->addChildItem(item);
    ItemTreeView::instance()->checkItem(item, true);
    // next, create link item under the joint item
    LinkItemPtr litem = new LinkItem(link);
    litem->originalNode = vloader->getOriginalNode(link);
    item->addChildItem(litem);
    SgNode* collisionShape = link->collisionShape();
    if (collisionShape != link->visualShape()) {
        LinkItemPtr citem = new LinkItem(link);
        citem->originalNode = vloader->getOriginalNode(link);
        citem->setName("collision");
        litem->addChildItem(citem);
    }
    ItemTreeView::instance()->checkItem(litem, true);

    if(link->child()){
        for(Link* child = link->child(); child; child = child->sibling()){
            setLinkTreeSub(child, vloader, item);
        }
    }
}

bool EditableModelItem::loadModelFile(const std::string& filename)
{
    return impl->loadModelFile(filename);
}


bool EditableModelItemImpl::loadModelFile(const std::string& filename)
{
    BodyPtr newBody;

    MessageView* mv = MessageView::instance();
    mv->beginStdioRedirect();

    bodyLoader.setMessageSink(mv->cout(true));
    newBody = bodyLoader.load(filename);

    mv->endStdioRedirect();
    
    if(newBody){
        newBody->initializeState();
        newBody->calcForwardKinematics();
        Link* link = newBody->rootLink();
        
        AbstractBodyLoaderPtr loader = bodyLoader.lastActualBodyLoader();
        VRMLBodyLoader* vloader = dynamic_cast<VRMLBodyLoader*>(loader.get());
        if (vloader) {
            // VRMLBodyLoader supports retriveOriginalNode function
            setLinkTree(link, vloader);
        } else {
            // Other loaders dont, so we wrap with inline node
            VRMLProtoInstance* proto = new VRMLProtoInstance(new VRMLProto(""));
            MFNode* children = new MFNode();
            VRMLInlinePtr inl = new VRMLInline();
            inl->urls.push_back(filename);
            children->push_back(inl);
            proto->fields["children"] = *children;
            // first, create joint item
            JointItemPtr item = new JointItem(link);
            item->originalNode = proto;
            self->addChildItem(item);
            ItemTreeView::instance()->checkItem(item, true);
            // next, create link item under the joint item
            LinkItemPtr litem = new LinkItem(link);
            litem->originalNode = proto;
            litem->setName("link");
            item->addChildItem(litem);
            ItemTreeView::instance()->checkItem(litem, true);
        }
        for (int i = 0; i < newBody->numDevices(); i++) {
            Device* dev = newBody->device(i);
            SensorItemPtr sitem = new SensorItem(dev);
            Item* parent = self->findItem<Item>(dev->link()->name());
            if (parent) {
                parent->addChildItem(sitem);
                ItemTreeView::instance()->checkItem(sitem, true);
            }
        }
    }

    return (newBody);
}


VRMLNodePtr EditableModelItemImpl::toVRML()
{
    VRMLHumanoidPtr node;
    node = new VRMLHumanoid();
    for(Item* child = self->childItem(); child; child = child->nextItem()){
        EditableModelBase* item = dynamic_cast<EditableModelBase*>(child);
        if (item) {
            VRMLNodePtr childnode = item->toVRML();
            node->humanoidBody.push_back(childnode);
        }
    }
    return node;
}


string EditableModelItemImpl::toURDF()
{
    ostringstream ss;
    ss << "<robot name=\"" << self->name() << "\">" << endl;
    for(Item* child = self->childItem(); child; child = child->nextItem()){
        EditableModelBase* item = dynamic_cast<EditableModelBase*>(child);
        if (item) {
            ss << item->toURDF();
        }
    }
    ss << "</robot>" << endl;
    return ss.str();
}


bool EditableModelItem::saveModelFile(const std::string& filename)
{
    return impl->saveModelFile(filename);
}


bool EditableModelItemImpl::saveModelFile(const std::string& filename)
{
    std::ofstream of;
    of.open(filename.c_str(), std::ios::out);
    VRMLBodyWriter* writer = new VRMLBodyWriter(of);
    writer->setOutFileName(filename);
    writer->writeHeader();

    of << endl;
    of << "PROTO Joint [" << endl;
    of << "  exposedField     SFVec3f      center              0 0 0" << endl;
    of << "  exposedField     MFNode       children            []" << endl;
    of << "  exposedField     MFFloat      llimit              []" << endl;
    of << "  exposedField     MFFloat      lvlimit             []" << endl;
    of << "  exposedField     SFRotation   limitOrientation    0 0 1 0" << endl;
    of << "  exposedField     SFString     name                \"\"" << endl;
    of << "  exposedField     SFRotation   rotation            0 0 1 0" << endl;
    of << "  exposedField     SFVec3f      scale               1 1 1" << endl;
    of << "  exposedField     SFRotation   scaleOrientation    0 0 1 0" << endl;
    of << "  exposedField     MFFloat      stiffness           [ 0 0 0 ]" << endl;
    of << "  exposedField     SFVec3f      translation         0 0 0" << endl;
    of << "  exposedField     MFFloat      ulimit              []" << endl;
    of << "  exposedField     MFFloat      uvlimit             []" << endl;
    of << "  exposedField     SFString     jointType           \"\"" << endl;
    of << "  exposedField     SFInt32      jointId             -1" << endl;
    of << "  exposedField     SFVec3f      jointAxis           0 0 1" << endl;
    of << endl;
    of << "  exposedField     SFFloat      gearRatio           1" << endl;
    of << "  exposedField     SFFloat      rotorInertia        0" << endl;
    of << "  exposedField     SFFloat      rotorResistor       0" << endl;
    of << "  exposedField     SFFloat      torqueConst         1" << endl;
    of << "  exposedField     SFFloat      encoderPulse        1" << endl;
    of << "]" << endl;
    of << "{" << endl;
    of << "  Transform {" << endl;
    of << "    center           IS center" << endl;
    of << "    children         IS children" << endl;
    of << "    rotation         IS rotation" << endl;
    of << "    scale            IS scale" << endl;
    of << "    scaleOrientation IS scaleOrientation" << endl;
    of << "    translation      IS translation" << endl;
    of << "  }" << endl;
    of << "}" << endl;
    of << endl;
    of << "PROTO Segment [" << endl;
    of << "  field           SFVec3f     bboxCenter        0 0 0" << endl;
    of << "  field           SFVec3f     bboxSize          -1 -1 -1" << endl;
    of << "  exposedField    SFVec3f     centerOfMass      0 0 0" << endl;
    of << "  exposedField    MFNode      children          [ ]" << endl;
    of << "  exposedField    SFNode      coord             NULL" << endl;
    of << "  exposedField    MFNode      displacers        [ ]" << endl;
    of << "  exposedField    SFFloat     mass              0" << endl;
    of << "  exposedField    MFFloat     momentsOfInertia  [ 0 0 0 0 0 0 0 0 0 ]" << endl;
    of << "  exposedField    SFString    name              \"\"" << endl;
    of << "  eventIn         MFNode      addChildren" << endl;
    of << "  eventIn         MFNode      removeChildren" << endl;
    of << "]" << endl;
    of << "{" << endl;
    of << "  Group {" << endl;
    of << "    addChildren    IS addChildren" << endl;
    of << "    bboxCenter     IS bboxCenter" << endl;
    of << "    bboxSize       IS bboxSize" << endl;
    of << "    children       IS children" << endl;
    of << "    removeChildren IS removeChildren" << endl;
    of << "  }" << endl;
    of << "}" << endl;
    of << endl;
    of << "PROTO Humanoid [" << endl;
    of << "  field           SFVec3f    bboxCenter            0 0 0" << endl;
    of << "  field           SFVec3f    bboxSize              -1 -1 -1" << endl;
    of << "  exposedField    SFVec3f    center                0 0 0" << endl;
    of << "  exposedField    MFNode     humanoidBody          [ ]" << endl;
    of << "  exposedField    MFString   info                  [ ]" << endl;
    of << "  exposedField    MFNode     joints                [ ]" << endl;
    of << "  exposedField    SFString   name                  \"\"" << endl;
    of << "  exposedField    SFRotation rotation              0 0 1 0" << endl;
    of << "  exposedField    SFVec3f    scale                 1 1 1" << endl;
    of << "  exposedField    SFRotation scaleOrientation      0 0 1 0" << endl;
    of << "  exposedField    MFNode     segments              [ ]" << endl;
    of << "  exposedField    MFNode     sites                 [ ]" << endl;
    of << "  exposedField    SFVec3f    translation           0 0 0" << endl;
    of << "  exposedField    SFString   version               \"1.1\"" << endl;
    of << "  exposedField    MFNode     viewpoints            [ ]" << endl;
    of << "]" << endl;
    of << "{" << endl;
    of << "  Transform {" << endl;
    of << "    bboxCenter       IS bboxCenter" << endl;
    of << "    bboxSize         IS bboxSize" << endl;
    of << "    center           IS center" << endl;
    of << "    rotation         IS rotation" << endl;
    of << "    scale            IS scale" << endl;
    of << "    scaleOrientation IS scaleOrientation" << endl;
    of << "    translation      IS translation" << endl;
    of << "    children [" << endl;
    of << "      Group {" << endl;
    of << "        children IS viewpoints" << endl;
    of << "      }" << endl;
    of << "    ]" << endl;
    of << "  }" << endl;
    of << "}" << endl;
    of << endl;
    of << "PROTO ExtraJoint [" << endl;
    of << "  exposedField SFString link1Name \"\"" << endl;
    of << "  exposedField SFString link2Name \"\"" << endl;
    of << "  exposedField SFVec3f  link1LocalPos 0 0 0" << endl;
    of << "  exposedField SFVec3f  link2LocalPos 0 0 0" << endl;
    of << "  exposedField SFString jointType \"xyz\"" << endl;
    of << "  exposedField SFVec3f  jointAxis 1 0 0" << endl;
    of << "]" << endl;
    of << "{" << endl;
    of << "}" << endl;
    of << endl;

    writer->writeNode(toVRML());
}


bool EditableModelItem::saveModelFileURDF(const std::string& filename)
{
    return impl->saveModelFileURDF(filename);
}


bool EditableModelItemImpl::saveModelFileURDF(const std::string& filename)
{
    std::ofstream of;
    of.open(filename.c_str(), std::ios::out);
    of << toURDF();
    of.close();
}


bool EditableModelItem::saveModelFileSDF(const std::string& filename)
{
    return impl->saveModelFileSDF(filename);
}


bool EditableModelItemImpl::saveModelFileSDF(const std::string& filename)
{
    sdf::SDFPtr robot(new sdf::SDF());
    sdf::init(robot);
    sdf::readString(toURDF(), robot);
    std::ofstream of;
    of.open(filename.c_str(), std::ios::out);
    of << robot->ToString();
    cout << robot->ToString() << endl;
    of.close();
}


Item* EditableModelItem::doDuplicate() const
{
    return new EditableModelItem(*this);
}


void EditableModelItem::doAssign(Item* srcItem)
{
    Item::doAssign(srcItem);
    impl->doAssign(srcItem);
}


void EditableModelItemImpl::doAssign(Item* srcItem)
{
    EditableModelItem* srcEditableModelItem = dynamic_cast<EditableModelItem*>(srcItem);
}


void EditableModelItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void EditableModelItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Model file"), getFilename(boost::filesystem::path(self->filePath())));
}


bool EditableModelItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool EditableModelItemImpl::store(Archive& archive)
{
    archive.writeRelocatablePath("modelFile", self->filePath());

    return true;
}


bool EditableModelItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool EditableModelItemImpl::restore(const Archive& archive)
{
    bool restored = false;
    
    string modelFile;
    if(archive.readRelocatablePath("modelFile", modelFile)){
        restored = self->load(modelFile);
    }

    return restored;
}
