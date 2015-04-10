/**
   @file
*/

#include "PrimitiveShapeItem.h"
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

namespace {

const bool TRACE_FUNCTIONS = false;

inline double radian(double deg) { return (3.14159265358979 * deg / 180.0); }

}


namespace cnoid {

class PrimitiveShapeItemImpl
{
public:
    PrimitiveShapeItem* self;
    Link* link;
    double mass;
    Vector3 centerOfMass;
    Matrix3 momentsOfInertia;
    Selection primitiveType;
    Vector3f primitiveColor;
    Vector3 boxSize;
    double primitiveRadius;
    double primitiveHeight;
    bool isselected;

    SgPosTransform* sceneLink;
    SgShape* shape;

    //ModelEditDraggerPtr positionDragger;
    PositionDraggerPtr positionDragger;

    PrimitiveShapeItemImpl(PrimitiveShapeItem* self);
    PrimitiveShapeItemImpl(PrimitiveShapeItem* self, Link* link);
    PrimitiveShapeItemImpl(PrimitiveShapeItem* self, const PrimitiveShapeItemImpl& org);
    ~PrimitiveShapeItemImpl();
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


void PrimitiveShapeItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        ext->itemManager().registerClass<PrimitiveShapeItem>(N_("PrimitiveShapeItem"));
        ext->itemManager().addCreationPanel<PrimitiveShapeItem>();
        initialized = true;
    }
}


PrimitiveShapeItem::PrimitiveShapeItem()
{
    impl = new PrimitiveShapeItemImpl(this);
}


PrimitiveShapeItemImpl::PrimitiveShapeItemImpl(PrimitiveShapeItem* self)
    : self(self)
{
    link = new Link();
    link->setName("Link");
    link->setShape(new SgPosTransform());
    init();
}


PrimitiveShapeItem::PrimitiveShapeItem(Link* link)
{
    impl = new PrimitiveShapeItemImpl(this, link);
}
    

PrimitiveShapeItemImpl::PrimitiveShapeItemImpl(PrimitiveShapeItem* self, Link* link)
{
    this->link = link;
    init();
}
    

PrimitiveShapeItem::PrimitiveShapeItem(const PrimitiveShapeItem& org)
    : Item(org)
{
    impl = new PrimitiveShapeItemImpl(this, *org.impl);
}


PrimitiveShapeItemImpl::PrimitiveShapeItemImpl(PrimitiveShapeItem* self, const PrimitiveShapeItemImpl& org)
    : self(self)
{
    link = org.link;
    init();
}


ItemPtr PrimitiveShapeItem::doDuplicate() const
{
    return new PrimitiveShapeItem(*this);
}


void PrimitiveShapeItem::doAssign(Item* srcItem)
{
    Item::doAssign(srcItem);
    impl->doAssign(srcItem);
}


void PrimitiveShapeItemImpl::doAssign(Item* srcItem)
{
    PrimitiveShapeItem* srcPrimitiveShapeItem = dynamic_cast<PrimitiveShapeItem*>(srcItem);
#if 0
    if(srcPrimitiveShapeItem){
        Link* srcLink = srcPrimitiveShapeItem->link();
        link->p() = srcLink->p();
        link->R() = srcLink->R();
    }
#endif
}


void PrimitiveShapeItemImpl::init()
{
    primitiveType.resize(4);
    primitiveType.setSymbol(0, "Box");
    primitiveType.setSymbol(1, "Sphere");
    primitiveType.setSymbol(2, "Cylinder");
    primitiveType.setSymbol(3, "Cone");
    primitiveType.select("Box");
    primitiveColor[0] = 0.5f;
    primitiveColor[1] = 0.5f;
    primitiveColor[2] = 0.5f;
    boxSize[0] = 0.1;
    boxSize[1] = 0.1;
    boxSize[2] = 0.1;
    primitiveRadius = 0.1;
    primitiveHeight = 0.1;

    mass = link->mass();
    centerOfMass = link->centerOfMass();
    momentsOfInertia = link->I();
    self->translation = link->translation();
    self->rotation = link->rotation();
    sceneLink = new SgPosTransform();
    shape = NULL;
    self->setName(link->name());

    attachPositionDragger();

    self->sigUpdated().connect(boost::bind(&PrimitiveShapeItemImpl::onUpdated, this));
    self->sigPositionChanged().connect(boost::bind(&PrimitiveShapeItemImpl::onPositionChanged, this));
    ItemTreeView::mainInstance()->sigSelectionChanged().connect(boost::bind(&PrimitiveShapeItemImpl::onSelectionChanged, this));
    isselected = false;

    onUpdated();
}


void PrimitiveShapeItemImpl::attachPositionDragger()
{
    positionDragger = new ModelEditDragger;
    positionDragger->sigDragStarted().connect(boost::bind(&PrimitiveShapeItemImpl::onDraggerStarted, this));
    positionDragger->sigPositionDragged().connect(boost::bind(&PrimitiveShapeItemImpl::onDraggerDragged, this));
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


void PrimitiveShapeItemImpl::onSelectionChanged()
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


void PrimitiveShapeItemImpl::onDraggerStarted()
{
}


void PrimitiveShapeItemImpl::onDraggerDragged()
{
    self->translation = positionDragger->draggedPosition().translation();
    self->rotation = positionDragger->draggedPosition().rotation();
    self->notifyUpdate();
}

PrimitiveShapeItem::~PrimitiveShapeItem()
{
    delete impl;
}


PrimitiveShapeItemImpl::~PrimitiveShapeItemImpl()
{
}


Link* PrimitiveShapeItem::link() const
{
    return impl->link;
}


void PrimitiveShapeItemImpl::onUpdated()
{
    sceneLink->translation() = self->translation;
    sceneLink->rotation() = self->rotation;
    string pt(primitiveType.selectedSymbol());
    if (shape) {
        sceneLink->removeChild(shape);
        shape = NULL;
    }
    shape = new SgShape;
    SgMaterial* material = new SgMaterial;
    material->setDiffuseColor(Vector3f::Zero());
    material->setEmissiveColor(primitiveColor);
    material->setAmbientIntensity(0.0f);
    material->setTransparency(0.0f);
    MeshGenerator meshGenerator;
    if (pt == "Box") {
        SgMeshPtr mesh = meshGenerator.generateBox(boxSize);
        shape->setMesh(mesh);
        shape->setMaterial(material);
    }
    if (pt == "Cone") {
        SgMeshPtr mesh = meshGenerator.generateCone(primitiveRadius, primitiveHeight, true, true);
        shape->setMesh(mesh);
        shape->setMaterial(material);
    }
    if (pt == "Cylinder") {
        SgMeshPtr mesh = meshGenerator.generateCylinder(primitiveRadius, primitiveHeight);
        shape->setMesh(mesh);
        shape->setMaterial(material);
    }
    if (pt == "Sphere") {
        SgMeshPtr mesh = meshGenerator.generateSphere(primitiveRadius);
        shape->setMesh(mesh);
        shape->setMaterial(material);
    }
    sceneLink->addChildOnce(shape);
    sceneLink->notifyUpdate();
}


void PrimitiveShapeItemImpl::onPositionChanged()
{
}


VRMLNodePtr PrimitiveShapeItem::toVRML()
{
    return impl->toVRML();
}

VRMLNodePtr PrimitiveShapeItemImpl::toVRML()
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
    VRMLShapePtr shape;
    shape = new VRMLShape();
    trans->children.push_back(shape);
    string pt(primitiveType.selectedSymbol());
    if (pt == "Box") {
        VRMLBoxPtr box = new VRMLBox();
        box->size = boxSize;
        shape->geometry = box;
    }
    if (pt == "Cone") {
        VRMLConePtr cone = new VRMLCone();
        cone->bottomRadius = primitiveRadius;
        cone->height = primitiveHeight;
        cone->bottom = true;
        cone->side = true;
        shape->geometry = cone;
    }
    if (pt == "Cylinder") {
        VRMLCylinderPtr cylinder = new VRMLCylinder();
        cylinder->radius = primitiveRadius;
        cylinder->height = primitiveHeight;
        cylinder->top = true;
        cylinder->bottom = true;
        cylinder->side = true;
        shape->geometry = cylinder;
    }
    if (pt == "Sphere") {
        VRMLSpherePtr sphere = new VRMLSphere();
        sphere->radius = primitiveRadius;
        shape->geometry = sphere;
    }
    return node;
}


bool PrimitiveShapeItemImpl::setPrimitiveType(const std::string& t)
{
    return primitiveType.select(t);
}


SgNode* PrimitiveShapeItem::getScene()
{
    return impl->sceneLink;
}


void PrimitiveShapeItem::doPutProperties(PutPropertyFunction& putProperty)
{
    EditableModelBase::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void PrimitiveShapeItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    ostringstream oss;
    putProperty.decimals(4)(_("Mass"), mass);
    putProperty(_("Center of mass"), str(Vector3(centerOfMass)),
                boost::bind(&PrimitiveShapeItemImpl::setCenterOfMass, this, _1));
    oss.str("");
    oss << momentsOfInertia;
    putProperty(_("Inertia"), oss.str(),
                boost::bind(&PrimitiveShapeItemImpl::setInertia, this, _1));
    putProperty(_("Primitive type"), primitiveType,
                boost::bind(&Selection::selectIndex, &primitiveType, _1));
    string pt(primitiveType.selectedSymbol());
    if (pt == "Box") {
        putProperty(_("Box size"), str(boxSize),
                    boost::bind(&PrimitiveShapeItemImpl::setBoxSize, this, _1));
    }
    if (pt == "Cone") {
        putProperty.decimals(4)(_("Cone radius"), primitiveRadius);
        putProperty.decimals(4)(_("Cone height"), primitiveHeight);
    }
    if (pt == "Cylinder") {
        putProperty.decimals(4)(_("Cylinder radius"), primitiveRadius);
        putProperty.decimals(4)(_("Cylinder height"), primitiveHeight);
    }
    if (pt == "Sphere") {
        putProperty.decimals(4)(_("Sphere radius"), primitiveRadius);
    }
    oss.str("");
    oss << primitiveColor;
    putProperty(_("Color"), oss.str(),
                boost::bind(&PrimitiveShapeItemImpl::setPrimitiveColor, this, _1));
}


bool PrimitiveShapeItemImpl::setCenterOfMass(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        centerOfMass = p;
        return true;
    }
    return false;
}


bool PrimitiveShapeItemImpl::setInertia(const std::string& value)
{
    return false;
}

bool PrimitiveShapeItemImpl::setBoxSize(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        boxSize = p;
        return true;
    }
    return false;
}


bool PrimitiveShapeItemImpl::setPrimitiveColor(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        primitiveColor[0] = p[0];
        primitiveColor[1] = p[1];
        primitiveColor[2] = p[2];
        return true;
    }
    return false;
}


bool PrimitiveShapeItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool PrimitiveShapeItemImpl::store(Archive& archive)
{
    archive.setDoubleFormat("% .6f");

    write(archive, "position", self->translation);
    write(archive, "attitude", Matrix3(self->rotation));

    return true;
}


bool PrimitiveShapeItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool PrimitiveShapeItemImpl::restore(const Archive& archive)
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
