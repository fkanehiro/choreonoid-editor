/**
   @file
*/

#include "EditableModelItem.h"
#include "EditableModelBase.h"
#include "JointItem.h"
#include "LinkItem.h"
#include "SensorItem.h"
#include "PrimitiveShapeItem.h"
#include "MeshShapeItem.h"
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
#include <boost/variant.hpp>
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
        ext->itemManager().registerClass<EditableModelItem>(N_("EditableBodyItem"));
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

void showSgNodeTree(SgNode *node, int indent)
{
    SgGroup *group = dynamic_cast<SgGroup *>(node);
    for (int i=0; i<indent; i++) std::cout << " ";
    if (group) {
        if (dynamic_cast<SgPosTransform*>(node)){
            std::cout << "SgPosTransform" << std::endl;
        }else{
            std::cout << "SgGroup" << std::endl;
        }
        for (int i=0; i<group->numChildren(); i++){
            showSgNodeTree(group->child(i), indent+2);
        }
    }else if(dynamic_cast<SgShape *>(node)){
        std::cout << "SgShape" << std::endl;
    }else{
        std::cout << "SgNode" << std::endl;
    }
}

void showVRMLNodeTree(VRMLNode *node, int indent=0)
{
    for (int i=0; i<indent; i++) std::cout << " ";

    VRMLInline *inlineNode = dynamic_cast<VRMLInline *>(node);
    if (inlineNode){
        std::cout << "VRMLInline" << std::endl;
        return;
    }

    VRMLProtoInstance *proto = dynamic_cast<VRMLProtoInstance *>(node);
    if (proto){
        std::cout << "VRMLProtoInstance(" << proto->proto->protoName << ")" << std::endl;
        if (proto->proto->protoName != "Segment") return;
        MFNode children = boost::get<MFNode>(proto->fields["children"]);
        for (unsigned int i=0; i<children.size(); i++){
            showVRMLNodeTree(children[i].get(), indent+2);
        }
        return;
    }

    VRMLTransform *trans = dynamic_cast<VRMLTransform *>(node);
    if (trans){
        std::cout << "VRMLTransform" << std::endl;
        for (int i=0; i<trans->countChildren(); i++){
            showVRMLNodeTree(trans->getChild(i), indent+2);
        }
        return;
    }

    VRMLGroup *group = dynamic_cast<VRMLGroup *>(node);
    if (group){
        std::cout << "VRMLGroup" << std::endl;
        for (int i=0; i<group->countChildren(); i++){
            showVRMLNodeTree(group->getChild(i), indent+2);
        }
        return;
    }

    VRMLShape *shape = dynamic_cast<VRMLShape *>(node);
    if (shape){
        std::cout << "VRMLShape" << std::endl;
        return;
    }

    std::cout << "VRMLNode" << std::endl;
}

void createShapeItems(LinkItem *linkItem, SgNode *snode, VRMLNode *vnode,
                      Vector3 translation, Matrix3 rotation)
{
    VRMLInline *inlineNode = dynamic_cast<VRMLInline *>(vnode);
    if (inlineNode){
        //std::cout << "VRMLInline" << std::endl;
        MeshShapeItemPtr mesh
            = new MeshShapeItem(translation, rotation, snode, inlineNode->urls[0]);
        linkItem->addChildItem(mesh);
        return;
    }

    VRMLProtoInstance *proto = dynamic_cast<VRMLProtoInstance *>(vnode);
    if (proto){
        //std::cout << "VRMLProtoInstance" << std::endl;
        if (proto->proto->protoName != "Segment") return;
        //std::cout << "VRMLSegment" << std::endl;
        MFNode children = boost::get<MFNode>(proto->fields["children"]);
        SgGroup *group = dynamic_cast<SgGroup *>(snode);
        if (!group){
            std::cerr << "error(1) in createShapeItems" << std::endl;
        }
        if (group->numChildren()!=1) {
            std::cerr << "error(2) in createShapeItems" << std::endl;
        }
        SgPosTransform *strans = dynamic_cast<SgPosTransform *>(group->child(0));
        if (strans){ // This means link->Rs() is not identity matrix
            translation = rotation*strans->translation() + translation;
            rotation = rotation*strans->rotation();
            if (strans->numChildren() != 1){
                std::cerr << "error(7) in createShapeItems" << std::endl;
            }
            group = strans;
        }
        group = dynamic_cast<SgGroup *>(group->child(0));
        if (!group){
            std::cerr << "error(3) in createShapeItems" << std::endl;
        }
        if (group->numChildren() != children.size()){
            std::cerr << "error(6) in createShapeItems" << std::endl;
        }
        for (unsigned int i=0; i<children.size(); i++){
            createShapeItems(linkItem, group->child(i), children[i].get(), translation, rotation);
        }
        return;
    }

    VRMLTransform *vtrans = dynamic_cast<VRMLTransform *>(vnode);
    if (vtrans){
        //std::cout << "VRMLTransform" << std::endl;
        SgPosTransform *strans = dynamic_cast<SgPosTransform *>(snode);
        if (!strans){
            std::cerr << "error(4) in createShapeItems" << std::endl;
        }
        if (vtrans->countChildren() != strans->numChildren()){
            std::cerr << "error(5) in createShapeItems" << std::endl;
        }
        translation = rotation*strans->translation() + translation;
        rotation = rotation*strans->rotation();
        for (int i=0; i<vtrans->countChildren(); i++){
            createShapeItems(linkItem, strans->child(i), vtrans->getChild(i),
                             translation, rotation);
        }
        return;
    }

    VRMLShape *vshape = dynamic_cast<VRMLShape *>(vnode);
    if (vshape){
        //std::cout << "VRMLShape" << std::endl;
        SgShape *sshape = dynamic_cast<SgShape *>(snode);
        if (!sshape){
            std::cerr << "error in createShapeItems" << std::endl;
        }
        SgMesh *mesh = sshape->mesh();
        if (mesh->primitiveType() != SgMesh::MESH){
            PrimitiveShapeItemPtr primitive
                = new PrimitiveShapeItem(translation, rotation, sshape);
            linkItem->addChildItem(primitive);
        }
        return;
    }
}

void EditableModelItemImpl::setLinkTreeSub(Link* link, VRMLBodyLoader* vloader, Item* parentItem)
{
    // first, create joint item
    JointItemPtr item = new JointItem(link);
    //item->originalNode = vloader->getOriginalNode(link);
    parentItem->addChildItem(item);
    ItemTreeView::instance()->checkItem(item, true);
    SgNode* visualShape = link->visualShape();
    link->setVisualShape(NULL);
    // next, create link item under the joint item
    LinkItemPtr litem = new LinkItem(link);
    //litem->originalNode = vloader->getOriginalNode(link);
    item->addChildItem(litem);
    SgNode* collisionShape = link->collisionShape();
    if (collisionShape != visualShape) {
        LinkItemPtr citem = new LinkItem(link);
        //citem->originalNode = vloader->getOriginalNode(link);
        citem->setName("collision");
        litem->addChildItem(citem);
    }
#if 0
    std::cout << link->name() << std::endl;
    showVRMLNodeTree(vloader->getOriginalNode(link).get());
    showSgNodeTree(visualShape,0);
#endif
    createShapeItems(litem, visualShape, vloader->getOriginalNode(link).get(),
                     Vector3::Zero(), link->Rs().transpose());
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
        JointItem *rootJoint = dynamic_cast<JointItem*>(self->childItem());
        if (rootJoint) rootJoint->updatePosition();
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

    writer->writeOpenHRPPROTOs();
    writer->writeNode(toVRML());

    return true;
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
    return true;
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
    return true;
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
