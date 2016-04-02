/**
   @file
*/

#include "EditableModelBase.h"
#include <cnoid/EigenArchive>
#include <cnoid/Archive>
#include <cnoid/VRML>
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

namespace cnoid{
vector<double> readvector(const std::string& value)
{
    double v;
    vector<double> ret;
    
    stringstream ss(value);
    
    while(!ss.eof()){
        ss >> v;
        ret.push_back(v);
        ss.ignore();
    }
    return ret;
}
}

EditableModelBase::EditableModelBase()
    : translation(),
      rotation()
{}


void EditableModelBase::doPutProperties(PutPropertyFunction& putProperty)
{
    ostringstream oss;
    putProperty(_("Translation"), str(Vector3(translation)),
                boost::bind(&EditableModelBase::onTranslationChanged, this, _1));
    SFRotation rotaxis;
    rotaxis = rotation;
    oss.str("");
    oss << rotaxis.angle() << " " << str(rotaxis.axis());
    putProperty(_("Rotation (Axis)"), oss.str(),
                boost::bind(&EditableModelBase::onRotationAxisChanged, this, _1));
    Vector3 rpy(rpyFromRot(rotation));
    putProperty("Rotation (RPY)", str(TO_DEGREE * rpy), boost::bind(&EditableModelBase::onRotationRPYChanged, this, _1));
    oss.str("");
    oss << rotation;
    putProperty(_("Rotation (Matrix)"), oss.str(),
                boost::bind(&EditableModelBase::onRotationChanged, this, _1));
}


bool EditableModelBase::onTranslationChanged(const std::string& value)
{
    Vector3 p;
    if(toVector3(value, p)){
        translation = p;
        return true;
    }
    return false;
}


bool EditableModelBase::onRotationChanged(const std::string& value)
{
    vector<double> v = readvector(value);
    if (v.size() != 9) {
        return false;
    }
    rotation << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return true;
}


bool EditableModelBase::onRotationAxisChanged(const std::string& value)
{
    vector<double> v = readvector(value);
    if (v.size() != 4) {
        return false;
    }
    SFRotation rot;
    SFRotation::Vector3& axis = rot.axis();
    axis[0] = v[1];
    axis[1] = v[2];
    axis[2] = v[3];
    rot.angle() = v[0];

    double size = axis.norm();
    if(size < 1.0e-6){
        return false;
    }
    axis /= size; // normalize
    
    rotation = rot.toRotationMatrix();
    return true;
}


bool EditableModelBase::onRotationRPYChanged(const std::string& value)
{
    Vector3 rpy;
    if(toVector3(value, rpy)){
        rotation = rotFromRpy(TO_RADIAN * rpy);
        return true;
    }
    return false;
}
