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
#include <cnoid/Sensor>
#include <cnoid/Camera>
#include <cnoid/RangeSensor>
#include <cnoid/VRMLBody>
#include "ModelEditDragger.h"
#include <cnoid/FileUtil>
#include <cnoid/MeshNormalGenerator>
#include <cnoid/MeshGenerator>
#include <cnoid/RangeCamera>
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
    Device* device;
    Selection sensorType;
    Selection cameraType;
    int resolutionX;
    int resolutionY;
    double nearDistance;
    double farDistance;
    double fieldOfView;
    double frameRate;
    Vector3 maxForce;
    Vector3 maxTorque;
    Vector3 maxAngularVelocity;
    Vector3 maxAcceleration;
    double scanAngle;
    double scanStep;
    double scanRate;
    double minDistance;
    double maxDistance;
    bool isselected;

    SceneLinkPtr sceneLink;
    SgScaleTransformPtr defaultAxesScale;
    SgMaterialPtr axisMaterials[3];
    double axisCylinderNormalizedRadius;
    SgPosTransformPtr sensorShape;

    ModelEditDraggerPtr positionDragger;

    SensorItemImpl(SensorItem* self);
    SensorItemImpl(SensorItem* self, Device* dev);
    SensorItemImpl(SensorItem* self, const SensorItemImpl& org);
    ~SensorItemImpl();
    
    void init();
    void syncDevice();
    void onSelectionChanged();
    void attachPositionDragger();
    void onDraggerStarted();
    void onDraggerDragged();
    void onUpdated();
    double radius() const;
    void setRadius(double val);
    bool onMaxForceChanged(const std::string& value);
    bool onMaxTorqueChanged(const std::string& value);
    bool onMaxAngularVelocityChanged(const std::string& value);
    bool onMaxAccelerationChanged(const std::string& value);
    VRMLNodePtr toVRML();
    string toURDF();
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
    device = new Camera();
    init();
}


SensorItem::SensorItem(Device *dev)
{
    impl = new SensorItemImpl(this, dev);
}


SensorItemImpl::SensorItemImpl(SensorItem* self, Device* dev)
    : self(self)
{
    this->device = dev;
    self->setName(dev->name());
    Affine3 position = dev->link()->position() * dev->T_local();
    self->translation = position.translation();
    self->rotation = position.rotation();
    init();
    syncDevice();
}


SensorItem::SensorItem(const SensorItem& org)
    : EditableModelBase(org)
{
    impl = new SensorItemImpl(this, *org.impl);
}


SensorItemImpl::SensorItemImpl(SensorItem* self, const SensorItemImpl& org)
    : self(self),
      device(org.device)
{
    init();
    syncDevice();
}


void SensorItemImpl::init()
{
    sensorType.resize(2);
    sensorType.setSymbol(0, "camera");
    sensorType.setSymbol(1, "range");
    sensorType.setSymbol(2, "force");
    sensorType.setSymbol(3, "acceleration");
    sensorType.setSymbol(4, "gyro");
    sensorType.select("camera");

    cameraType.resize(3);
    cameraType.setSymbol(0, "NONE");
    cameraType.setSymbol(1, "COLOR");
    cameraType.setSymbol(2, "DEPTH");
    cameraType.setSymbol(3, "COLOR_DEPTH");
    cameraType.setSymbol(4, "POINT_CLOUD");
    cameraType.setSymbol(5, "COLOR_POINT_CLOUD");
    cameraType.select("COLOR");

    sceneLink = new SceneLink(new Link());
    sensorShape = NULL;

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

    attachPositionDragger();
    
    setRadius(0.15);

    self->sigUpdated().connect(boost::bind(&SensorItemImpl::onUpdated, this));
    ItemTreeView::mainInstance()->sigSelectionChanged().connect(boost::bind(&SensorItemImpl::onSelectionChanged, this));
    isselected = false;

    onUpdated();
}

void SensorItemImpl::syncDevice()
{
    ForceSensor* fsensor = dynamic_cast<ForceSensor*>(device);
    if (fsensor) {
        sensorType.select("force");
        maxForce = fsensor->F_max().head<3>();
        maxTorque = fsensor->F_max().tail<3>();
    }
    RateGyroSensor* gyro = dynamic_cast<RateGyroSensor*>(device);
    if (gyro) {
        sensorType.select("gyro");
        maxAngularVelocity = gyro->w_max();
    }
    AccelerationSensor* asensor = dynamic_cast<AccelerationSensor*>(device);
    if (asensor) {
        sensorType.select("acceleration");
        maxAcceleration = asensor->dv_max();
    }
    RangeSensor* rsensor = dynamic_cast<RangeSensor*>(device);
    if (rsensor) {
        sensorType.select("range");
        if (rsensor->yawRange() > 0)
            scanAngle = rsensor->yawRange();
        else
            scanAngle = rsensor->pitchRange();
        scanStep = rsensor->pitchStep();
        scanRate = rsensor->frameRate();
        minDistance = rsensor->minDistance();
        maxDistance = rsensor->maxDistance();
    }
    Camera* camera = dynamic_cast<Camera*>(device);
    if (camera) {
        sensorType.select("camera");
        RangeCamera* range = dynamic_cast<RangeCamera*>(device);
        if (range) {
            if (range->isOrganized()) {
                if (range->imageType() == Camera::NO_IMAGE) {
                    cameraType.select("DEPTH");
                } else {
                    cameraType.select("COLOR_DEPTH");
                }
            } else {
                if (range->imageType() == Camera::NO_IMAGE) {
                    cameraType.select("POINT_CLOUD");
                } else {
                    cameraType.select("COLOR_POINT_CLOUD");
                }
            }
        } else {
            if (camera->imageType() == Camera::COLOR_IMAGE) {
                cameraType.select("COLOR");
            } else {
                cameraType.select("NONE");
            }
        }
        resolutionX = camera->resolutionX();
        resolutionY = camera->resolutionY();
        frameRate = camera->frameRate();
        fieldOfView = camera->fieldOfView();
        nearDistance = camera->nearDistance();
        farDistance = camera->farDistance();
    }
    onUpdated();
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
        //positionDragger->setDraggerAlwaysShown(selected);
        if (isselected) {
            positionDragger->setDraggerAlwaysShown(true);
        } else {
            positionDragger->setDraggerAlwaysHidden(true);
        }
    }
}


double SensorItemImpl::radius() const
{
    return defaultAxesScale->scale().x();
}


void SensorItemImpl::setRadius(double r)
{
    defaultAxesScale->setScale(r);
    positionDragger->setRadius(r * 1.5);
    sceneLink->notifyUpdate();
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
    self->translation = positionDragger->draggedPosition().translation();
    self->rotation = positionDragger->draggedPosition().rotation();
    self->notifyUpdate();
 }

SensorItem::~SensorItem()
{
    delete impl;
}


SensorItemImpl::~SensorItemImpl()
{
}


Device* SensorItem::device() const
{
    return impl->device;
}


void SensorItemImpl::onUpdated()
{
    sceneLink->translation() = self->translation;
    sceneLink->rotation() = self->rotation;

    // draw shape indicator for sensors
    if (sensorShape) {
        sceneLink->removeChild(sensorShape);
        sensorShape = NULL;
    }
    string st(sensorType.selectedSymbol());
    if (st == "camera") {
        sensorShape = new SgPosTransform;
        SgShapePtr shape = new SgShape;
        SgMaterialPtr material = new SgMaterial;
        material->setDiffuseColor(Vector3f(0.0f, 0.0f, 1.0f));
        material->setEmissiveColor(Vector3f::Zero());
        material->setAmbientIntensity(0.0f);
        material->setTransparency(0.5f);
        
        double d = 0.50;
        double w = 0.50;
        double h = 0.40;
        if (fieldOfView > 0.0 && resolutionX > 0.0 && resolutionY > 0.0) {
            double aspect = (double)resolutionX / (double)resolutionY;
            if (aspect >= 1.0) {
                w = 2.0 * d * tan(fieldOfView / 2.0);
                h = w / aspect;
            } else {
                h = 2.0 * d * tan(fieldOfView / 2.0);
                w = h * aspect;
            }
        }
        SgMeshPtr mesh = new SgMesh;
        SgVertexArray& vertices = *mesh->setVertices(new SgVertexArray());
        vertices.reserve(5);
        vertices.push_back(Vector3f(   0,    0,  0));
        vertices.push_back(Vector3f(-w/2, -h/2, -d));
        vertices.push_back(Vector3f(-w/2,  h/2, -d));
        vertices.push_back(Vector3f( w/2,  h/2, -d));
        vertices.push_back(Vector3f( w/2, -h/2, -d));
        
        mesh->reserveNumTriangles(4);
        mesh->addTriangle(0,1,2);
        mesh->addTriangle(0,2,3);
        mesh->addTriangle(0,3,4);
        mesh->addTriangle(0,4,1);

        MeshNormalGenerator normalGenerator;
        normalGenerator.generateNormals(mesh, 0);
        
        mesh->updateBoundingBox();
        
        shape->setMesh(mesh);
        shape->setMaterial(material);
        sensorShape->addChild(shape);
        sceneLink->addChildOnce(sensorShape);
    }else if (st == "range") {
        sensorShape = new SgPosTransform;
        SgShapePtr shape = new SgShape;
        SgMaterialPtr material = new SgMaterial;
        material->setDiffuseColor(Vector3f(0.0f, 0.0f, 1.0f));
        material->setEmissiveColor(Vector3f::Zero());
        material->setAmbientIntensity(0.0f);
        material->setTransparency(0.5f);
        
        double d = 0.50;
        double w = 2.0 * d * tan(scanAngle / 2.0);
        SgMeshPtr mesh = new SgMesh;
        SgVertexArray& vertices = *mesh->setVertices(new SgVertexArray());
        vertices.reserve(3);
        vertices.push_back(Vector3f(   0, 0,  0));
        vertices.push_back(Vector3f(-w/2, 0, -d));
        vertices.push_back(Vector3f( w/2, 0, -d));
        
        mesh->reserveNumTriangles(1);
        mesh->addTriangle(0,1,2);

        MeshNormalGenerator normalGenerator;
        normalGenerator.generateNormals(mesh, 0);
        
        mesh->updateBoundingBox();
        
        shape->setMesh(mesh);
        shape->setMaterial(material);
        sensorShape->addChild(shape);
        sceneLink->addChildOnce(sensorShape);
    }
    sceneLink->notifyUpdate();
}


Item* SensorItem::doDuplicate() const
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
        Link* srcLink = srcSensorItem->device()->link();
        device->link()->p() = srcLink->p();
        device->link()->R() = srcLink->R();
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
        putProperty.decimals(4).min(0)(_("Resolution X"), resolutionX, changeProperty(resolutionX));
        putProperty.decimals(4).min(0)(_("Resolution Y"), resolutionY, changeProperty(resolutionY));
        putProperty.decimals(4).min(0)(_("Frame rate"), frameRate, changeProperty(frameRate));
        putProperty.decimals(4)(_("Field of view"), fieldOfView, changeProperty(fieldOfView));
        putProperty.decimals(4)(_("Near distance"), nearDistance, changeProperty(nearDistance));
        putProperty.decimals(4)(_("Far distance"), farDistance, changeProperty(farDistance));
    } else if (st == "force") {
        putProperty("Max force", str(maxForce), boost::bind(&SensorItemImpl::onMaxForceChanged, this, _1));
        putProperty("Max torque", str(maxTorque), boost::bind(&SensorItemImpl::onMaxTorqueChanged, this, _1));
    } else if (st == "gyro") {
        putProperty("Max angular velocity", str(maxAngularVelocity), boost::bind(&SensorItemImpl::onMaxAngularVelocityChanged, this, _1));
    } else if (st == "acceleration") {
        putProperty("Max acceleration", str(maxAcceleration), boost::bind(&SensorItemImpl::onMaxAccelerationChanged, this, _1));
    } else if (st == "range") {
        putProperty("Scan angle", scanAngle, changeProperty(scanAngle));
        putProperty("Scan step", scanStep, changeProperty(scanStep));
        putProperty("Scan rate", scanRate, changeProperty(scanRate));
        putProperty("Min distance", minDistance, changeProperty(minDistance));
        putProperty("Max distance", maxDistance, changeProperty(maxDistance));
    }
}

bool SensorItemImpl::onMaxForceChanged(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        maxForce = p;
        return true;
    }
    return false;
}

bool SensorItemImpl::onMaxTorqueChanged(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        maxTorque = p;
        return true;
    }
    return false;
}

bool SensorItemImpl::onMaxAngularVelocityChanged(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        maxAngularVelocity = p;
        return true;
    }
    return false;
}

bool SensorItemImpl::onMaxAccelerationChanged(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        maxAcceleration = p;
        return true;
    }
    return false;
}

VRMLNodePtr SensorItem::toVRML()
{
    return impl->toVRML();
}


VRMLNodePtr SensorItemImpl::toVRML()
{
    VRMLNodePtr node = NULL;
    string st(sensorType.selectedSymbol());
    if (st == "force") {
        VRMLForceSensorPtr fnode = new VRMLForceSensor();
        fnode->maxForce = maxForce;
        fnode->maxTorque = maxTorque;
        node = fnode;
    } else if (st == "gyro") {
        VRMLGyroPtr gnode = new VRMLGyro();
        gnode->maxAngularVelocity = maxAngularVelocity;
        node = gnode;
    } else if (st == "acceleration") {
        VRMLAccelerationSensorPtr anode = new VRMLAccelerationSensor();
        anode->maxAcceleration = maxAcceleration;
        node = anode;
    } else if (st == "range") {
        VRMLRangeSensorPtr rnode = new VRMLRangeSensor();
        rnode->scanAngle = scanAngle;
        rnode->scanStep = scanStep;
        rnode->scanRate = scanRate;
        rnode->minDistance = minDistance;
        rnode->maxDistance = maxDistance;
        node = rnode;
    } else if (st == "camera") {
        VRMLVisionSensorPtr cnode = new VRMLVisionSensor();
        cnode->type = cameraType.selectedSymbol();
        cnode->width = resolutionX;
        cnode->height = resolutionY;
        cnode->frameRate = frameRate;
        cnode->fieldOfView = fieldOfView;
        cnode->frontClipDistance = nearDistance;
        cnode->backClipDistance = farDistance;
        node = cnode;
    }
    return node;
}


string SensorItem::toURDF()
{
    return impl->toURDF();
}


string SensorItemImpl::toURDF()
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
