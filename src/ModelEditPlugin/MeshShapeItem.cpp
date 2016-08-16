/**
   @file
*/

#include "MeshShapeItem.h"
#include "JointItem.h"
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
#include <cnoid/BodyState>
#include <cnoid/SceneBody>
#include <cnoid/VRML>
#include <cnoid/VRMLBody>
#include <cnoid/MeshGenerator>
#include "ModelEditDragger.h"
#include <cnoid/FileUtil>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <bitset>
#include <deque>
#include <iostream>
#include <sstream>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

inline double radian(double deg) { return (3.14159265358979 * deg / 180.0); }

}


namespace cnoid {

class MeshShapeItemImpl
{
public:
    MeshShapeItem* self;
    std::string path;
    bool isselected;

    SgPosTransform* sceneLink;
    SgNode* shape;

    //ModelEditDraggerPtr positionDragger;
    PositionDraggerPtr positionDragger;
    Connection conSelectUpdate;

    MeshShapeItemImpl(MeshShapeItem* self);
    MeshShapeItemImpl(MeshShapeItem* self, const MeshShapeItemImpl& org);
    ~MeshShapeItemImpl();
    void doAssign(Item* srcItem);
        
    void init();
    void attachPositionDragger();
    void onDraggerStarted();
    void onDraggerDragged();
    void onUpdated();
    void onPositionChanged();
    void onSelectionChanged();
    void doPutProperties(PutPropertyFunction& putProperty);
    VRMLNodePtr toVRML();
    string toURDF();
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}


void MeshShapeItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        ext->itemManager().registerClass<MeshShapeItem>(N_("MeshShapeItem"));
        ext->itemManager().addCreationPanel<MeshShapeItem>();
        initialized = true;
    }
}


MeshShapeItem::MeshShapeItem()
{
    impl = new MeshShapeItemImpl(this);
}


MeshShapeItemImpl::MeshShapeItemImpl(MeshShapeItem* self)
    : self(self)
{
    init();
}


MeshShapeItem::MeshShapeItem(const MeshShapeItem& org)
    : EditableModelBase(org)
{
    impl = new MeshShapeItemImpl(this, *org.impl);
}


MeshShapeItemImpl::MeshShapeItemImpl(MeshShapeItem* self, const MeshShapeItemImpl& org)
    : self(self)
{
    init();
}


Item* MeshShapeItem::doDuplicate() const
{
    return new MeshShapeItem(*this);
}


void MeshShapeItem::doAssign(Item* srcItem)
{
    Item::doAssign(srcItem);
    impl->doAssign(srcItem);
}


void MeshShapeItemImpl::doAssign(Item* srcItem)
{
    MeshShapeItem* srcMeshShapeItem = dynamic_cast<MeshShapeItem*>(srcItem);
#if 0
    if(srcMeshShapeItem){
        Link* srcLink = srcMeshShapeItem->link();
        link->p() = srcLink->p();
        link->R() = srcLink->R();
    }
#endif
}


void MeshShapeItemImpl::init()
{
    sceneLink = new SgPosTransform();
    shape = NULL;

    attachPositionDragger();

    self->sigUpdated().connect(boost::bind(&MeshShapeItemImpl::onUpdated, this));
    self->sigPositionChanged().connect(boost::bind(&MeshShapeItemImpl::onPositionChanged, this));
    conSelectUpdate = ItemTreeView::mainInstance()->sigSelectionChanged().connect(boost::bind(&MeshShapeItemImpl::onSelectionChanged, this));
    isselected = false;

    onUpdated();
}


void MeshShapeItemImpl::attachPositionDragger()
{
    positionDragger = new ModelEditDragger;
    positionDragger->sigDragStarted().connect(boost::bind(&MeshShapeItemImpl::onDraggerStarted, this));
    positionDragger->sigPositionDragged().connect(boost::bind(&MeshShapeItemImpl::onDraggerDragged, this));
    BoundingBox bb = sceneLink->untransformedBoundingBox();
    if (bb.empty()) {
        positionDragger->setRadius(0.1);
    } else {
        positionDragger->adjustSize(sceneLink->untransformedBoundingBox());
    }
    sceneLink->addChild(positionDragger);
    sceneLink->notifyUpdate();
    self->notifyUpdate();
}


void MeshShapeItemImpl::onSelectionChanged()
{
    ItemList<Item> items = ItemTreeView::mainInstance()->selectedItems();
    bool selected = false;
    for(size_t i=0; i < items.size(); ++i){
        Item* item = items.get(i);
        if (item == self) {
            selected = true;
        }
    }
    if (isselected != selected) {
        isselected = selected;
        //positionDragger->setDraggerAlwaysShown(selected);
        if (isselected) {
            positionDragger->setDraggerAlwaysShown(true);
        } else {
            positionDragger->setDraggerAlwaysHidden(true);
        }
    }
}


void MeshShapeItemImpl::onDraggerStarted()
{
}


void MeshShapeItemImpl::onDraggerDragged()
{
    self->translation = positionDragger->draggedPosition().translation();
    self->rotation = positionDragger->draggedPosition().rotation();
    self->notifyUpdate();
}

MeshShapeItem::~MeshShapeItem()
{
    delete impl;
}


MeshShapeItemImpl::~MeshShapeItemImpl()
{
    conSelectUpdate.disconnect();
}


void MeshShapeItemImpl::onUpdated()
{
    sceneLink->translation() = self->absTranslation;
    sceneLink->rotation() = self->absRotation;
    if (shape) {
        sceneLink->removeChild(shape);
        shape = NULL;
    }
    BodyLoader bodyLoader;
    BodyPtr newBody = bodyLoader.load(path);
    if (!newBody) return;
    shape = newBody->rootLink()->visualShape();
    sceneLink->addChildOnce(shape);
    sceneLink->notifyUpdate();
}


void MeshShapeItemImpl::onPositionChanged()
{
}


VRMLNodePtr MeshShapeItem::toVRML()
{
    return impl->toVRML();
}

VRMLNodePtr MeshShapeItemImpl::toVRML()
{
    VRMLTransformPtr trans;
    trans = new VRMLTransform();
    trans->translation = self->translation;
    trans->rotation = self->rotation;
    VRMLInlinePtr inlineNode;
    inlineNode = new VRMLInline();
    inlineNode->urls.push_back(path);
    trans->children.push_back(inlineNode);
    return trans;
}


string MeshShapeItem::toURDF()
{
    return impl->toURDF();
}

string MeshShapeItemImpl::toURDF()
{
    ostringstream ss;
    ss << "<link name=\"" << self->name() << "\">" << endl;
    Affine3 relative;
    JointItem* parentjoint = dynamic_cast<JointItem*>(self->parentItem());
    if (parentjoint) {
        Affine3 parent, child;
        parent.translation() = parentjoint->translation;
        parent.linear() = parentjoint->rotation;
        child.translation() = self->translation;
        child.linear() = self->rotation;
        relative = parent.inverse() * child;
    } else {
        relative.translation() = self->translation;
        relative.linear() = self->rotation;
    }
    for (int i=0; i < 2; i++) {
        if (i == 0) {
            ss << " <visual>" << endl;
        } else {
            ss << " <collision>" << endl;
        }
        if (i == 0) {
            ss << " </visual>" << endl;
        } else {
            ss << " </collision>" << endl;
        }
    }
    return ss.str();
}


SgNode* MeshShapeItem::getScene()
{
    return impl->sceneLink;
}


void MeshShapeItem::doPutProperties(PutPropertyFunction& putProperty)
{
    EditableModelBase::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void MeshShapeItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Path"), path, changeProperty(path));
}


bool MeshShapeItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool MeshShapeItemImpl::store(Archive& archive)
{
    archive.setDoubleFormat("% .6f");

    write(archive, "position", self->translation);
    write(archive, "attitude", Matrix3(self->rotation));

    return true;
}


bool MeshShapeItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool MeshShapeItemImpl::restore(const Archive& archive)
{
    Vector3 p;
    if(read(archive, "position", p)){
        self->translation = p;
    }
    Matrix3 R;
    if(read(archive, "attitude", R)){
        self->rotation = R;
    }

    return true;
}
