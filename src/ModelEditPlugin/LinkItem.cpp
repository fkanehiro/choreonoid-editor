/**
   @file
*/

#include "LinkItem.h"
#include "JointItem.h"
#include <cnoid/EigenArchive>
#include <cnoid/Archive>
#include <cnoid/ItemManager>
#include <cnoid/ItemTreeView>
#include <cnoid/SceneBody>
#include <cnoid/VRMLBody>
#include <cnoid/VRMLWriter>
#include <cnoid/MeshGenerator>
#include "ModelEditDragger.h"
#include <boost/bind.hpp>
#include <iostream>
#include <sstream>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using namespace boost;

namespace {

const bool TRACE_FUNCTIONS = false;

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

    SceneLink* sceneLink;
    SgNode* mesh;
    SgShape* shape;
    SgPosTransformPtr massShape;
    bool visualizeMass;

    Vector3 dragStartTranslation;
    //ModelEditDraggerPtr positionDragger;
    PositionDraggerPtr positionDragger;
    Connection conSelectUpdate;

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
    VRMLNodePtr toVRML();
    string toURDF();
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
    : EditableModelBase(org)
{
    impl = new LinkItemImpl(this, *org.impl);
}


LinkItemImpl::LinkItemImpl(LinkItem* self, const LinkItemImpl& org)
    : self(self)
{
    link = org.link;
    init();
}


Item* LinkItem::doDuplicate() const
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
    centerOfMass = link->Rs().transpose()*link->centerOfMass();
    momentsOfInertia = link->Rs().transpose()*link->I()*link->Rs();
    sceneLink = new SceneLink(link);
    massShape = NULL;
    visualizeMass = false;

    if(self->name().size() == 0){
        self->setName(link->name() + "_LINK");
    }

    attachPositionDragger();

    self->sigUpdated().connect(boost::bind(&LinkItemImpl::onUpdated, this));
    self->sigPositionChanged().connect(boost::bind(&LinkItemImpl::onPositionChanged, this));
    conSelectUpdate = ItemTreeView::mainInstance()->sigSelectionChanged().connect(boost::bind(&LinkItemImpl::onSelectionChanged, this));
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
    dragStartTranslation = positionDragger->draggedPosition().translation();
}


void LinkItemImpl::onDraggerDragged()
{
    self->translation = positionDragger->draggedPosition().translation();
    self->rotation = positionDragger->draggedPosition().rotation();
    self->notifyUpdate();
}

void LinkItemImpl::onUpdated()
{
    sceneLink->translation() = self->absTranslation;
    sceneLink->rotation() = self->absRotation;
    
    // draw shape indicator for mass
    if (massShape) {
        sceneLink->removeChild(massShape);
        massShape = NULL;
    }
    if (visualizeMass) {
        // hide original mesh
        sceneLink->setVisible(false);
        // generate CoM ball
        massShape = new SgPosTransform;
        shape = new SgShape;
        SgMaterialPtr commaterial = new SgMaterial;
        commaterial->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));
        commaterial->setEmissiveColor(Vector3f::Zero());
        commaterial->setAmbientIntensity(0.0f);
        commaterial->setTransparency(0.0f);
        MeshGenerator meshGenerator;
        SgMeshPtr mesh = meshGenerator.generateSphere(0.02);
        shape->setMesh(mesh);
        shape->setMaterial(commaterial);
        massShape->addChild(shape);
        // generate inertia shape
        SgShapePtr inertshape = new SgShape;
        SgScaleTransformPtr inertscale = new SgScaleTransform;
        SgMaterialPtr inertmaterial = new SgMaterial;
        inertmaterial->setDiffuseColor(Vector3f(0.0f, 0.0f, 1.0f));
        inertmaterial->setEmissiveColor(Vector3f::Zero());
        inertmaterial->setAmbientIntensity(0.0f);
        inertmaterial->setTransparency(0.5f);
        SgMeshPtr inertmesh = meshGenerator.generateSphere(0.1);
        inertshape->setMesh(inertmesh);
        inertshape->setMaterial(inertmaterial);
        inertscale->addChild(inertshape);
        Matrix3 moiinv = momentsOfInertia.inverse();
        inertscale->setScale(Vector3(moiinv(0,0), moiinv(1,1), moiinv(2,2)).normalized());
        massShape->addChild(inertscale);
        
        massShape->setTranslation(centerOfMass);
        sceneLink->addChildOnce(massShape);
    } else {
        sceneLink->setVisible(true);
    }
    sceneLink->notifyUpdate();
}

LinkItem::~LinkItem()
{
    delete impl;
}


LinkItemImpl::~LinkItemImpl()
{
    conSelectUpdate.disconnect();
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
    node->defName = self->name();
    if (self->originalNode) {
        VRMLProtoInstancePtr original = dynamic_pointer_cast<VRMLProtoInstance>(self->originalNode);
        if (original) {
            node->children = get<MFNode>(original->fields["children"]);
        }
    }
    for(Item* child = self->childItem(); child; child = child->nextItem()){
        EditableModelBase* item = dynamic_cast<EditableModelBase*>(child);
        if (item){
            node->children.push_back(item->toVRML());
        }
    }
    return node;
}


string LinkItem::toURDF()
{
    return impl->toURDF();
}

string LinkItemImpl::toURDF()
{
    ostringstream ss;
    JointItem* parentjoint = dynamic_cast<JointItem*>(self->parentItem());
    Affine3 relative;
    if (parentjoint) {
        string meshfname = "";
        Assimp::Importer* im;
        std::stringstream vrml;
        if (self->originalNode) {
            VRMLProtoInstancePtr original = dynamic_pointer_cast<VRMLProtoInstance>(self->originalNode);
            if (original) {
                VRMLWriter* writer = new VRMLWriter(vrml);
                writer->setOutFileName("temp");
                writer->writeNode(original);
            }
        }
        const aiScene* ashape;
        ashape = im->ReadFileFromMemory(vrml.str().c_str(), vrml.str().length(), 0);
        Assimp::Exporter* ex;
        ex = new Assimp::Exporter();
        ex->Export(ashape, "collada", meshfname + ".dae");
        ex->Export(ashape, "stl", meshfname + ".stl");
        Affine3 parent, child;
        parent.translation() = parentjoint->translation;
        parent.linear() = parentjoint->rotation;
        child.translation() = self->translation;
        child.linear() = self->rotation;
        relative = parent.inverse() * child;
        ss << "<link name=\"" << parentjoint->name() << "_LINK\">" << endl;
        ss << " <inertial>" << endl;
        ss << "  <mass value=\"" << mass << "\"/>" << endl;
        ss << "  <origin xyz=\"" << centerOfMass[0] << " " << centerOfMass[1] << " " << centerOfMass[1] << "\" rpy=\"0 0 0\"/>" << endl;
        ss << "  <inertia ixx=\"" << momentsOfInertia(0, 0)
           << "\" ixy=\"" << momentsOfInertia(0, 1)
           << "\" ixz=\"" << momentsOfInertia(0, 2)
           << "\" iyy=\"" << momentsOfInertia(1, 1)
           << "\" iyz=\"" << momentsOfInertia(1, 2)
           << "\" izz=\"" << momentsOfInertia(2, 2) << "\" />" << endl;
        ss << " </inertial>" << endl;
        ss << " <visual>" << endl;
        ss << "  <geometry>" << endl;
        ss << "   <mesh filename=\"" << meshfname << ".dae\" />" << endl;
        ss << "  </geometry>" << endl;
        ss << " </visual>" << endl;
        ss << " <collision>" << endl;
        ss << "  <geometry>" << endl;
        ss << "   <mesh filename=\"" << meshfname << ".stl\" />" << endl;
        ss << "  </geometry>" << endl;
        ss << " </collision>" << endl;
        ss << "</link>" << endl;
    }
    return ss.str();
}


SgNode* LinkItem::getScene()
{
    return impl->sceneLink;
}


void LinkItem::doPutProperties(PutPropertyFunction& putProperty)
{
    //EditableModelBase::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void LinkItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    ostringstream oss;
    //putProperty(_("Model name"), link->name());
    //putProperty(_("Model file"), getFilename(boost::filesystem::path(self->filePath())));
    putProperty.decimals(4)(_("Mass"), mass, changeProperty(mass));
    putProperty(_("Center of mass"), str(Vector3(centerOfMass)),
                boost::bind(&LinkItemImpl::setCenterOfMass, this, _1));
    oss.str("");
    oss << momentsOfInertia;
    putProperty(_("Inertia"), oss.str(),
                boost::bind(&LinkItemImpl::setInertia, this, _1));
    putProperty.decimals(4)(_("Visualize mass"), visualizeMass, changeProperty(visualizeMass));
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
    vector<double> v = readvector(value);
    if (v.size() != 9) {
        return false;
    }
    momentsOfInertia << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return true;
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
