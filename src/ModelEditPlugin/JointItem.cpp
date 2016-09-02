/**
   @file
*/

#include "JointItem.h"
#include <cnoid/EigenArchive>
#include <cnoid/Archive>
#include <cnoid/ItemTreeView>
#include <cnoid/ItemManager>
#include <cnoid/VRMLBody>
#include <cnoid/SceneBody>
#include "ModelEditDragger.h"
#include <cnoid/MeshGenerator>
#include <boost/bind.hpp>
#include <iostream>
#include <sstream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

const char* axisNames[3] = { "x", "y", "z" };

const bool TRACE_FUNCTIONS = false;

inline double radian(double deg) { return (3.14159265358979 * deg / 180.0); }

}


namespace cnoid {

class JointItemImpl
{
public:
    JointItem* self;
    LinkPtr link;
    int jointId;
    Selection jointType;
    Vector3 jointAxis;
    double ulimit;
    double llimit;
    double uvlimit;
    double lvlimit;
    double gearRatio;
    double rotorInertia;
    double rotorResistor;
    double torqueConst;
    double encoderPulse;
    bool isselected;

    SceneLinkPtr sceneLink;
    SgScaleTransformPtr defaultAxesScale;
    SgMaterialPtr axisMaterials[3];
    double axisCylinderNormalizedRadius;
    SgPosTransformPtr axisShape;

    Vector3 prevDragTranslation;
    //ModelEditDraggerPtr positionDragger;
    PositionDraggerPtr positionDragger;
    Connection conSelectUpdate;

    JointItemImpl(JointItem* self);
    JointItemImpl(JointItem* self, Link* link);
    JointItemImpl(JointItem* self, const JointItemImpl& org);
    ~JointItemImpl();
    
    void init();
    void onSelectionChanged();
    void attachPositionDragger();
    void onDraggerStarted();
    void onDraggerDragged();
    void onDraggerDraggedRecur(Item *parent, Vector3 dragdiff);
    void onUpdated();
    void onPositionChanged();
    double radius() const;
    void setRadius(double val);
    VRMLNodePtr toVRML();
    string toURDF();
    void doAssign(Item* srcItem);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool setJointAxis(const std::string& value);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};

}
    

void JointItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        ext->itemManager().registerClass<JointItem>(N_("JointItem"));
        ext->itemManager().addCreationPanel<JointItem>();
        initialized = true;
    }
}


JointItem::JointItem()
{
    impl = new JointItemImpl(this);
}


JointItemImpl::JointItemImpl(JointItem* self)
    : self(self)
{
    link = new Link();
    init();
}


JointItem::JointItem(Link* link)
{
    impl = new JointItemImpl(this, link);
}


JointItemImpl::JointItemImpl(JointItem*self, Link* link)
    : self(self)
{
    this->link = link;
    init();
}


JointItem::JointItem(const JointItem& org)
    : EditableModelBase(org)
{
    impl = new JointItemImpl(this, *org.impl);
}


JointItemImpl::JointItemImpl(JointItem* self, const JointItemImpl& org)
    : self(self),
      link(org.link)
{
    init();
}


void JointItemImpl::init()
{
    jointType.resize(4);
    jointType.setSymbol(Link::ROTATIONAL_JOINT, "rotate");
    jointType.setSymbol(Link::SLIDE_JOINT, "slide");
    jointType.setSymbol(Link::FREE_JOINT, "free");
    jointType.setSymbol(Link::FIXED_JOINT, "fixed");
    jointType.setSymbol(Link::CRAWLER_JOINT, "crawler");

    jointId = link->jointId();
    jointType.selectIndex(link->jointType());
    jointAxis = link->Rs().transpose()*link->jointAxis();
    ulimit = link->q_upper();
    llimit = link->q_lower();
    uvlimit = link->dq_upper();
    lvlimit = link->dq_lower();
    gearRatio = link->info<double>("gearRatio", 1);
    rotorInertia = link->info<double>("rotorInertia", 0);
    rotorResistor = link->info<double>("rotorResistor", 0);
    torqueConst = link->info<double>("torqueConst", 1);
    encoderPulse = link->info<double>("encoderPulse", 1);
    
    self->translation = link->b();
    self->rotation = link->Rs();
    sceneLink = new SceneLink(new Link());

    if (self->name().size() == 0)
        self->setName(link->name());

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

    self->sigUpdated().connect(boost::bind(&JointItemImpl::onUpdated, this));
    self->sigPositionChanged().connect(boost::bind(&JointItemImpl::onPositionChanged, this));
    conSelectUpdate = ItemTreeView::mainInstance()->sigSelectionChanged().connect(boost::bind(&JointItemImpl::onSelectionChanged, this));
    isselected = false;

    onUpdated();
}


void JointItemImpl::onSelectionChanged()
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


double JointItemImpl::radius() const
{
    return defaultAxesScale->scale().x();
}


void JointItemImpl::setRadius(double r)
{
    defaultAxesScale->setScale(r);
    positionDragger->setRadius(r * 1.5);
    sceneLink->notifyUpdate();
}


void JointItemImpl::attachPositionDragger()
{
    positionDragger = new ModelEditDragger;
    positionDragger->sigDragStarted().connect(boost::bind(&JointItemImpl::onDraggerStarted, this));
    positionDragger->sigPositionDragged().connect(boost::bind(&JointItemImpl::onDraggerDragged, this));
    positionDragger->adjustSize(sceneLink->untransformedBoundingBox());
    sceneLink->addChild(positionDragger);
    sceneLink->notifyUpdate();
    self->notifyUpdate();
}


void JointItemImpl::onDraggerStarted()
{
    prevDragTranslation = positionDragger->draggedPosition().translation();
}


void JointItemImpl::onDraggerDraggedRecur(Item *parent, Vector3 dragdiff)
{
    for(Item* child = parent->childItem(); child; child = child->nextItem()){
        EditableModelBase* item = dynamic_cast<EditableModelBase*>(child);
        if (item) {
            item->translation += dragdiff;
            item->notifyUpdate();
            onDraggerDraggedRecur(child, dragdiff);
        }
    }
}

void JointItemImpl::onDraggerDragged()
{
    self->translation = positionDragger->draggedPosition().translation();
    self->rotation = positionDragger->draggedPosition().rotation();
    Vector3 dragdiff = self->translation - prevDragTranslation;
    onDraggerDraggedRecur(self, dragdiff);
    prevDragTranslation = self->translation;
    self->notifyUpdate();
}

void JointItemImpl::onUpdated()
{
    sceneLink->translation() = self->absTranslation;
    sceneLink->rotation() = self->absRotation;

    // draw shape indicator for joint axis
    if (axisShape) {
        sceneLink->removeChild(axisShape);
        axisShape = NULL;
    }
    string jt(jointType.selectedSymbol());
    if (jt != "free" && jt != "fixed") {
        axisShape = new SgPosTransform;
        SgShapePtr shape = new SgShape;
        SgMaterialPtr material = new SgMaterial;
        material->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));
        material->setEmissiveColor(Vector3f::Zero());
        material->setAmbientIntensity(0.0f);
        material->setTransparency(0.0f);
        MeshGenerator meshGenerator;
        SgMeshPtr mesh = meshGenerator.generateDisc(0.15, 0.12);
        shape->setMesh(mesh);
        shape->setMaterial(material);
        axisShape->addChild(shape);
        Vector3 axis = jointAxis.cross(Vector3::UnitZ());
        double th = asin(axis.norm());
        axisShape->setRotation(AngleAxis(th, axis));
        sceneLink->addChildOnce(axisShape);
    }
    sceneLink->notifyUpdate();

}


JointItem::~JointItem()
{
    delete impl;
}


JointItemImpl::~JointItemImpl()
{
    conSelectUpdate.disconnect();
}


Link* JointItem::link() const
{
    return impl->link;
}


Item* JointItem::doDuplicate() const
{
    return new JointItem(*this);
}


void JointItem::doAssign(Item* srcItem)
{
    Item::doAssign(srcItem);
    impl->doAssign(srcItem);
}


void JointItemImpl::doAssign(Item* srcItem)
{
    JointItem* srcJointItem = dynamic_cast<JointItem*>(srcItem);
#if 0
    if(srcJointItem){
        Link* srcLink = srcJointItem->link();
        link->p() = srcLink->p();
        link->R() = srcLink->R();
    }
#endif
}


SgNode* JointItem::getScene()
{
    return impl->sceneLink;
}


void JointItem::doPutProperties(PutPropertyFunction& putProperty)
{
    EditableModelBase::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void JointItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    ostringstream oss;
    putProperty.decimals(4)(_("Joint ID"), jointId, changeProperty(jointId));
    putProperty(_("Joint type"), jointType,
                boost::bind(&Selection::selectIndex, &jointType, _1));
    string jt(jointType.selectedSymbol());
    if (jt == "rotate" || jt == "slide") {
        putProperty(_("Joint axis"), str(jointAxis),
                    boost::bind(&JointItemImpl::setJointAxis, this, _1));
        putProperty.decimals(4)(_("Upper limit"), ulimit, changeProperty(ulimit));
        putProperty.decimals(4)(_("Lower limit"), llimit, changeProperty(llimit));
        putProperty.decimals(4)(_("Upper velocity limit"), uvlimit, changeProperty(uvlimit));
        putProperty.decimals(4)(_("Lower velocity limit"), lvlimit, changeProperty(lvlimit));
        putProperty.decimals(4)(_("Gear ratio"), gearRatio, changeProperty(gearRatio));
        putProperty.decimals(4)(_("Rotor inertia"), rotorInertia, changeProperty(rotorInertia));
        putProperty.decimals(4)(_("Rotor resistor"), rotorResistor, changeProperty(rotorResistor));
        putProperty.decimals(4)(_("Torque const"), torqueConst, changeProperty(torqueConst));
        putProperty.decimals(4)(_("Encoder pulse"), encoderPulse, changeProperty(encoderPulse));
    }
    putProperty.decimals(4).min(0.0)(_("Axis size"), radius(),
                                     boost::bind(&JointItemImpl::setRadius, this, _1), true);
}


bool JointItemImpl::setJointAxis(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        jointAxis = p;
        return true;
    }
    return false;
}


void JointItemImpl::onPositionChanged()
{
    self->updatePosition();
}

VRMLNodePtr JointItem::toVRML()
{
    return impl->toVRML();
}

VRMLNodePtr JointItemImpl::toVRML()
{
    VRMLJointPtr node;
    node = new VRMLJoint();
    node->defName = self->name();
    node->jointId = jointId;
    node->jointType = jointType.selectedSymbol();
    node->jointAxis = jointAxis;
    node->ulimit.clear();
    node->ulimit.push_back(ulimit);
    node->llimit.clear();
    node->llimit.push_back(llimit);
    node->uvlimit.clear();
    node->uvlimit.push_back(uvlimit);
    node->lvlimit.clear();
    node->lvlimit.push_back(lvlimit);
    node->gearRatio = gearRatio;
    node->rotorInertia = rotorInertia;
    node->rotorResistor = rotorResistor;
    node->torqueConst = torqueConst;
    node->encoderPulse = encoderPulse;
    node->translation = self->translation;
    node->rotation = self->rotation;
    for(Item* child = self->childItem(); child; child = child->nextItem()){
        EditableModelBase* item = dynamic_cast<EditableModelBase*>(child);
        if (item) {
            VRMLNodePtr childnode = item->toVRML();
            node->children.push_back(childnode);
        }
    }
    return node;
}

string JointItem::toURDF()
{
    return impl->toURDF();
}

string JointItemImpl::toURDF()
{
    ostringstream ss;
    string jtype;
    jtype = "fixed";
    if (jointType.selectedSymbol() == "rotate") {
        // TODO: use continuous when no limits are set
        jtype = "revolute";
    } else if (jointType.selectedSymbol() == "slide") {
        jtype = "prismatic";
    }
    ss << "<joint name=\"" << self->name() << "\" type=\"" << jtype << "\">" << endl;
    ss << " <axis>" << jointAxis[0] << " " << jointAxis[1] << " " << jointAxis[2] << "</axis>" << endl;
    if (jtype == "revolute" || jtype == "prismatic") {
        ss << " <limit>" << endl;
        ss << "  <lower>" << llimit << "</lower>"<< endl;
        ss << "  <upper>" << ulimit << "</upper>"<< endl;
        ss << " </limit>" << endl;
    }
    JointItem* parentjoint = dynamic_cast<JointItem*>(self->parentItem());
    bool needworld = false;
    if (parentjoint) {
        Affine3 parent, child, relative;
        ss << " <parent link=\"" << parentjoint->name() << "_LINK\"/>" << endl;
        parent.translation() = parentjoint->translation;
        parent.linear() = parentjoint->rotation;
        child.translation() = self->translation;
        child.linear() = self->rotation;
        relative = parent.inverse() * child;
        Vector3 trans = relative.translation();
        Vector3 rpy = rpyFromRot(relative.rotation());
        ss << " <origin xyz=\"" << trans[0] << " " << trans[1] << " " << trans[2]
           << "\" rpy=\"" << rpy[0] << " " << rpy[1] << " " << rpy[2] << "\"/>" << endl;
    } else {
        ss << " <parent link=\"world\"/>" << endl;
        needworld = true;
    }
    ss << " <child link=\"" << self->name() << "_LINK\"/>" << endl;
    ss << "</joint>" << endl;
    if (needworld) {
        ss << "<link name=\"world\" />" << endl;
    }
    for(Item* child = self->childItem(); child; child = child->nextItem()){
        EditableModelBase* item = dynamic_cast<EditableModelBase*>(child);
        if (item) {
            ss << item->toURDF();
        }
    }
    return ss.str();
}

bool JointItem::store(Archive& archive)
{
    return impl->store(archive);
}


bool JointItemImpl::store(Archive& archive)
{
    archive.setDoubleFormat("% .6f");

    write(archive, "position", self->translation);
    write(archive, "attitude", Matrix3(self->rotation));

    return true;
}


bool JointItem::restore(const Archive& archive)
{
    return impl->restore(archive);
}


bool JointItemImpl::restore(const Archive& archive)
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
