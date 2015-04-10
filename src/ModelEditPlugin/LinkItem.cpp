/**
   @file
*/

#include "LinkItem.h"
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
#include <cnoid/MeshGenerator>
#include "ModelEditDragger.h"
#include <cnoid/FileUtil>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <bitset>
#include <deque>
#include <iostream>
#include <algorithm>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace boost;

namespace {

const bool TRACE_FUNCTIONS = false;

BodyLoader bodyLoader;

inline double radian(double deg) { return (3.14159265358979 * deg / 180.0); }

}


namespace cnoid {

class LinkItemImpl
{
public:
    LinkItem* self;
    Link* link;
    double mass;
    Vector3 centerOfMass;
    Matrix3 momentsOfInertia;
    bool isselected;

    SgPosTransform* sceneLink;
    SgNode* mesh;
    SgShape* shape;

    //ModelEditDraggerPtr positionDragger;
    PositionDraggerPtr positionDragger;

    LinkItemImpl(LinkItem* self);
    LinkItemImpl(LinkItem* self, Link* link);
    LinkItemImpl(LinkItem* self, const LinkItemImpl& org);
    ~LinkItemImpl();
    void doAssign(Item* srcItem);
        
    void init();
    void attachPositionDragger();
    void onDraggerStarted();
    void onDraggerDragged();
    void onUpdated();
    void onPositionChanged();
    void onSelectionChanged();
    void doPutProperties(PutPropertyFunction& putProperty);
    bool setCenterOfMass(const std::string& v);
    bool setInertia(const std::string& v);
    bool setPrimitiveType(const std::string& t);
    bool setBoxSize(const std::string& v);
    bool setPrimitiveColor(const std::string& v);
    VRMLNodePtr toVRML();
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}


void LinkItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        ext->itemManager().registerClass<LinkItem>(N_("LinkItem"));
        ext->itemManager().addCreationPanel<LinkItem>();
        initialized = true;
    }
}


LinkItem::LinkItem()
{
    impl = new LinkItemImpl(this);
}


LinkItemImpl::LinkItemImpl(LinkItem* self)
    : self(self)
{
    link = new Link();
    link->setName("Link");
    link->setShape(new SgPosTransform());
    init();
}


LinkItem::LinkItem(Link* link)
{
    impl = new LinkItemImpl(this, link);
}
    

LinkItemImpl::LinkItemImpl(LinkItem* self, Link* link)
    : self(self)
{
    this->link = link;
    init();
}


LinkItem::LinkItem(const LinkItem& org)
    : Item(org)
{
    impl = new LinkItemImpl(this, *org.impl);
}


LinkItemImpl::LinkItemImpl(LinkItem* self, const LinkItemImpl& org)
    : self(self)
{
    link = org.link;
    init();
}


ItemPtr LinkItem::doDuplicate() const
{
    return new LinkItem(*this);
}


void LinkItem::doAssign(Item* srcItem)
{
    Item::doAssign(srcItem);
    impl->doAssign(srcItem);
}


void LinkItemImpl::doAssign(Item* srcItem)
{
    LinkItem* srcLinkItem = dynamic_cast<LinkItem*>(srcItem);
#if 0
    if(srcLinkItem){
        Link* srcLink = srcLinkItem->link();
        link->p() = srcLink->p();
        link->R() = srcLink->R();
    }
#endif
}


void LinkItemImpl::init()
{
    mass = link->mass();
    centerOfMass = link->centerOfMass();
    momentsOfInertia = link->I();
    self->translation = link->translation();
    self->rotation = link->rotation();
    sceneLink = new SceneLink(link);
    self->setName(link->name());

    attachPositionDragger();

    self->sigUpdated().connect(boost::bind(&LinkItemImpl::onUpdated, this));
    self->sigPositionChanged().connect(boost::bind(&LinkItemImpl::onPositionChanged, this));
    ItemTreeView::mainInstance()->sigSelectionChanged().connect(boost::bind(&LinkItemImpl::onSelectionChanged, this));
    isselected = false;

    onUpdated();
}


void LinkItemImpl::attachPositionDragger()
{
    positionDragger = new ModelEditDragger;
    positionDragger->sigDragStarted().connect(boost::bind(&LinkItemImpl::onDraggerStarted, this));
    positionDragger->sigPositionDragged().connect(boost::bind(&LinkItemImpl::onDraggerDragged, this));
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


void LinkItemImpl::onSelectionChanged()
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


void LinkItemImpl::onDraggerStarted()
{
}


void LinkItemImpl::onDraggerDragged()
{
    self->translation = positionDragger->draggedPosition().translation();
    self->rotation = positionDragger->draggedPosition().rotation();
    self->notifyUpdate();
}

void LinkItemImpl::onUpdated()
{
    sceneLink->translation() = self->translation;
    sceneLink->rotation() = self->rotation;
    sceneLink->notifyUpdate();
}

LinkItem::~LinkItem()
{
    delete impl;
}


LinkItemImpl::~LinkItemImpl()
{
}


Link* LinkItem::link() const
{
    return impl->link;
}


void LinkItemImpl::onPositionChanged()
{
}


VRMLNodePtr LinkItem::toVRML()
{
    return impl->toVRML();
}

VRMLNodePtr LinkItemImpl::toVRML()
{
    VRMLSegmentPtr node;
    node = new VRMLSegment();
    node->mass = mass;
    node->centerOfMass = centerOfMass;
    MFFloat v(momentsOfInertia.data(), momentsOfInertia.data() + 9);
    node->momentsOfInertia = v;
    VRMLTransformPtr trans;
    trans = new VRMLTransform();
    JointItem* parentjoint = dynamic_cast<JointItem*>(self->parentItem());
    if (parentjoint) {
        node->defName = parentjoint->name() + "_LINK";
        Affine3 parent, child, relative;
        parent.translation() = parentjoint->translation;
        parent.linear() = parentjoint->rotation;
        child.translation() = self->translation;
        child.linear() = self->rotation;
        relative = parent.inverse() * child;
        trans->translation = relative.translation();
        trans->rotation = relative.rotation();
    } else {
        node->defName = self->name();
        trans->translation = self->translation;
        trans->rotation = self->rotation;
    }
    node->children.push_back(trans);
    if (self->loader) {
        VRMLProtoInstancePtr original = dynamic_pointer_cast<VRMLProtoInstance>(self->loader->retriveOriginalNode(link));
        if (original) {
            trans->children = get<MFNode>(original->fields["children"]);
        }
    }
    return node;
}


SgNode* LinkItem::getScene()
{
    return impl->sceneLink;
}


void LinkItem::doPutProperties(PutPropertyFunction& putProperty)
{
    EditableModelBase::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void LinkItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    ostringstream oss;
    //putProperty(_("Model name"), link->name());
    //putProperty(_("Model file"), getFilename(boost::filesystem::path(self->filePath())));
    putProperty.decimals(4)(_("Mass"), mass);
    putProperty(_("Center of mass"), str(Vector3(centerOfMass)),
                boost::bind(&LinkItemImpl::setCenterOfMass, this, _1));
    oss.str("");
    oss << momentsOfInertia;
    putProperty(_("Inertia"), oss.str(),
                boost::bind(&LinkItemImpl::setInertia, this, _1));
}


bool LinkItemImpl::setCenterOfMass(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        centerOfMass = p;
        return true;
    }
    return false;
}


bool LinkItemImpl::setInertia(const std::string& value)
{
    return false;
}


bool LinkItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool LinkItemImpl::store(Archive& archive)
{
    archive.setDoubleFormat("% .6f");

    archive.writeRelocatablePath("modelFile", self->filePath());

    write(archive, "position", link->p());
    write(archive, "attitude", Matrix3(link->R()));

    return true;
}


bool LinkItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool LinkItemImpl::restore(const Archive& archive)
{
    bool restored = false;
    
    string modelFile;
    if(archive.readRelocatablePath("modelFile", modelFile)){
        //restored = self->load(modelFile);
    }

    if(restored){
        Vector3 p;
        if(read(archive, "position", p)){
            link->p() = p;
        }
        Matrix3 R;
        if(read(archive, "attitude", R)){
            link->R() = R;
        }
    }

    return restored;
}
