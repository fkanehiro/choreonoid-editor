/**
   @file
*/

#include "SensorItem.h"
#include <cnoid/YAMLReader>
#include <cnoid/EigenArchive>
#include <cnoid/Archive>
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/LazySignal>
#include <cnoid/LazyCaller>
#include <cnoid/MessageView>
#include <cnoid/ItemManager>
#include <cnoid/OptionManager>
#include <cnoid/MenuManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/JointPath>
#include <cnoid/BodyLoader>
#include <cnoid/BodyState>
#include <cnoid/SceneBody>
#include <cnoid/SceneShape>
#include <cnoid/Camera>
#include "ModelEditDragger.h"
#include <cnoid/FileUtil>
#include <cnoid/MeshGenerator>
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

const char* axisNames[3] = { "x", "y", "z" };

const bool TRACE_FUNCTIONS = false;

inline double radian(double deg) { return (3.14159265358979 * deg / 180.0); }

}


namespace cnoid {

class SensorItemImpl
{
public:
    SensorItem* self;
    Link* link;
    Selection sensorType;
    Selection cameraType;
    bool isselected;

    SceneLinkPtr sceneLink;
    SgScaleTransformPtr defaultAxesScale;
    SgMaterialPtr axisMaterials[3];
    double axisCylinderNormalizedRadius;

    ModelEditDraggerPtr positionDragger;

    SensorItemImpl(SensorItem* self);
    SensorItemImpl(SensorItem* self, Link* link);
    SensorItemImpl(SensorItem* self, const SensorItemImpl& org);
    ~SensorItemImpl();
    
    void init();
    void onSelectionChanged();
    void attachPositionDragger();
    void onDraggerStarted();
    void onDraggerDragged();
    void onPositionChanged();
    double radius() const;
    void setRadius(double val);
    VRMLNodePtr toVRML(VRMLBodyLoader& loader);
    void doAssign(Item* srcItem);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}
    

void SensorItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        ext->itemManager().registerClass<SensorItem>(N_("SensorItem"));
        ext->itemManager().addCreationPanel<SensorItem>();
        initialized = true;
    }
}


SensorItem::SensorItem()
{
    impl = new SensorItemImpl(this);
}


SensorItemImpl::SensorItemImpl(SensorItem* self)
    : self(self)
{
    link = new Link();
    init();
}


SensorItem::SensorItem(Link *link)
{
    impl = new SensorItemImpl(this, link);
}


SensorItemImpl::SensorItemImpl(SensorItem* self, Link* link)
    : self(self)
{
    this->link = link;
    init();
    sceneLink->setPosition(link->position());
    sceneLink->notifyUpdate();
    self->setName(link->name());
}


SensorItem::SensorItem(const SensorItem& org)
    : Item(org)
{
    impl = new SensorItemImpl(this, *org.impl);
}


SensorItemImpl::SensorItemImpl(SensorItem* self, const SensorItemImpl& org)
    : self(self),
      link(org.link)
{
    init();
}


void SensorItemImpl::init()
{
    sensorType.resize(2);
    sensorType.setSymbol(0, "camera");
    sensorType.setSymbol(1, "force");
    sensorType.select("camera");

    cameraType.resize(3);
    cameraType.setSymbol(Camera::NO_IMAGE, "no");
    cameraType.setSymbol(Camera::COLOR_IMAGE, "color");
    cameraType.setSymbol(Camera::GRAYSCALE_IMAGE, "grayscale");

    sceneLink = new SceneLink(new Link());

    axisCylinderNormalizedRadius = 0.04;
    
    defaultAxesScale = new SgScaleTransform;
    
    for(int i=0; i < 3; ++i){
        SgMaterial* material = new SgMaterial;
        Vector3f color(0.2f, 0.2f, 0.2f);
        color[i] = 1.0f;
        material->setDiffuseColor(Vector3f::Zero());
        material->setEmissiveColor(color);
        material->setAmbientIntensity(0.0f);
        material->setTransparency(0.6f);
        axisMaterials[i] = material;
    }

    MeshGenerator meshGenerator;
    SgMeshPtr mesh = meshGenerator.generateArrow(1.8, 0.08, 0.1, 2.5);
    for(int i=0; i < 3; ++i){
        SgShape* shape = new SgShape;
        shape->setMesh(mesh);
        shape->setMaterial(axisMaterials[i]);
        
        SgPosTransform* arrow = new SgPosTransform;
        arrow->addChild(shape);
        if(i == 0){
            arrow->setRotation(AngleAxis(-PI / 2.0, Vector3::UnitZ()));
        } else if(i == 2){
            arrow->setRotation(AngleAxis( PI / 2.0, Vector3::UnitX()));
        }
        SgInvariantGroup* invariant = new SgInvariantGroup;
        invariant->setName(axisNames[i]);
        invariant->addChild(arrow);
        defaultAxesScale->addChild(invariant);
    }
    sceneLink->addChild(defaultAxesScale);

    setRadius(0.3);

    attachPositionDragger();
    
    self->sigPositionChanged().connect(boost::bind(&SensorItemImpl::onPositionChanged, this));
    ItemTreeView::mainInstance()->sigSelectionChanged().connect(boost::bind(&SensorItemImpl::onSelectionChanged, this));
    isselected = false;
}


void SensorItemImpl::onSelectionChanged()
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
        positionDragger->setDraggerAlwaysShown(selected);
    }
}


double SensorItemImpl::radius() const
{
    return defaultAxesScale->scale().x();
}


void SensorItemImpl::setRadius(double r)
{
    defaultAxesScale->setScale(r);
}


void SensorItemImpl::attachPositionDragger()
{
    positionDragger = new ModelEditDragger;
    positionDragger->sigDragStarted().connect(boost::bind(&SensorItemImpl::onDraggerStarted, this));
    positionDragger->sigPositionDragged().connect(boost::bind(&SensorItemImpl::onDraggerDragged, this));
    positionDragger->adjustSize(sceneLink->untransformedBoundingBox());
    sceneLink->addChild(positionDragger);
    sceneLink->notifyUpdate();
    self->notifyUpdate();
}


void SensorItemImpl::onDraggerStarted()
{
}


void SensorItemImpl::onDraggerDragged()
{
    link->position() = positionDragger->draggedPosition();
    sceneLink->setPosition(link->position());
    sceneLink->notifyUpdate();
    self->notifyUpdate();
 }

SensorItem::~SensorItem()
{
    delete impl;
}


SensorItemImpl::~SensorItemImpl()
{
}


Link* SensorItem::link() const
{
    return impl->link;
}


void SensorItemImpl::onPositionChanged()
{
}


ItemPtr SensorItem::doDuplicate() const
{
    return new SensorItem(*this);
}


void SensorItem::doAssign(Item* srcItem)
{
    Item::doAssign(srcItem);
    impl->doAssign(srcItem);
}


void SensorItemImpl::doAssign(Item* srcItem)
{
    SensorItem* srcSensorItem = dynamic_cast<SensorItem*>(srcItem);
    if(srcSensorItem){
        Link* srcLink = srcSensorItem->link();
        link->p() = srcLink->p();
        link->R() = srcLink->R();
    }
}


SgNode* SensorItem::getScene()
{
    return impl->sceneLink;
}


void SensorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    impl->doPutProperties(putProperty);
}


void SensorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    ostringstream oss;
    putProperty(_("Translation"), str(Vector3(sceneLink->translation())),
                boost::bind(&SensorItem::onTranslationChanged, self, _1));
    SFRotation rotation;
    rotation = sceneLink->rotation();
    oss.str("");
    oss << rotation.angle() << " " << str(rotation.axis());
    putProperty(_("Rotation (Axis)"), oss.str(),
                boost::bind(&SensorItem::onRotationAxisChanged, self, _1));
    Vector3 rpy(rpyFromRot(sceneLink->rotation()));
    putProperty("Rotation (RPY)", str(TO_DEGREE * rpy), boost::bind(&SensorItem::onRotationRPYChanged, self, _1));
    oss.str("");
    oss << sceneLink->rotation();
    putProperty(_("Rotation (Matrix)"), oss.str(),
                boost::bind(&SensorItem::onRotationChanged, self, _1));
    putProperty(_("Sensor type"), sensorType,
                boost::bind(&Selection::selectIndex, &sensorType, _1));
    string st(sensorType.selectedSymbol());
    if (st == "camera") {
        putProperty(_("Camera type"), cameraType,
                    boost::bind(&Selection::selectIndex, &cameraType, _1));
    }
}


VRMLNodePtr SensorItem::toVRML(VRMLBodyLoader& loader)
{
    return impl->toVRML(loader);
}


VRMLNodePtr SensorItemImpl::toVRML(VRMLBodyLoader& loader)
{
    return NULL;
}


bool SensorItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool SensorItemImpl::store(Archive& archive)
{
    archive.setDoubleFormat("% .6f");

    write(archive, "position", sceneLink->translation());
    write(archive, "attitude", Matrix3(sceneLink->rotation()));

    return true;
}


bool SensorItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool SensorItemImpl::restore(const Archive& archive)
{
    Vector3 p;
    if(read(archive, "position", p)){
        sceneLink->translation() = p;
    }
    Matrix3 R;
    if(read(archive, "attitude", R)){
        sceneLink->rotation() = R;
    }

    return true;
}
