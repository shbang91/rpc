#include "pnc/rolling_joint/rolling_joint.hpp"


using namespace dart;
using namespace dart::dynamics;

RJUniqueProperties::RJUniqueProperties(
        const Eigen::Vector3d& axis, 
        const Eigen::Vector3d& j1_axis_offset, 
        const double& r0_r1_ratio)
    : m_axis(axis.normalized())
    , m_j1_axis_offset(j1_axis_offset)
      , m_r0_r1_ratio(r0_r1_ratio)
{
    // orthogonalize axes
    double dot_product = m_axis.dot(m_j1_axis_offset);
    assert(std::abs(dot_product) < 1.0 - 1e-6);
    if (std::abs(dot_product) > 1e-6)
        m_j1_axis_offset = (m_j1_axis_offset - dot_product * m_axis).normalized();
}


RJProperties::RJProperties(
        const GenericJoint<math::R1Space>::Properties& genericJointProperties,
        const RJUniqueProperties& plProperties)
    : GenericJoint<math::R1Space>::Properties(genericJointProperties),
    RJUniqueProperties(plProperties)
{
    // Do nothing
}

RollingJoint::~RollingJoint()
{
    // Do nothing
}

void RollingJoint::setProperties(const Properties& _properties)
{
    GenericJoint<math::R1Space>::setProperties(static_cast<const GenericJoint<math::R1Space>::Properties&>(_properties));
    setProperties(static_cast<const UniqueProperties&>(_properties));
}

void RollingJoint::setProperties(const UniqueProperties& _properties)
{
    setAspectProperties(_properties);
}

void RollingJoint::setAspectProperties(const AspectProperties& properties)
{
    mAspectProperties.m_axis = properties.m_axis;
    mAspectProperties.m_j1_axis_offset = properties.m_j1_axis_offset;
    mAspectProperties.m_r0_r1_ratio = properties.m_r0_r1_ratio;
}


RollingJoint::Properties RollingJoint::getRJProperties() const
{
    return Properties(getGenericJointProperties(), mAspectProperties);
}

void RollingJoint::copy(const RollingJoint& _otherJoint)
{
    if(this == &_otherJoint)
        return;
    setProperties(_otherJoint.getRJProperties());
}

void RollingJoint::copy(const RollingJoint* _otherJoint)
{
    if(nullptr == _otherJoint)
        return;
    copy(*_otherJoint);
}

RollingJoint& RollingJoint::operator=(const RollingJoint& _otherJoint)
{
    copy(_otherJoint);
    return *this;
}

const std::string& RollingJoint::getType() const
{
    return getStaticType();
}

const std::string& RollingJoint::getStaticType()
{
    static const std::string name = "RollingJoint";
    return name;
}

bool RollingJoint::isCyclic(std::size_t /*_index*/) const
{
    return false;
}


RollingJoint::RollingJoint(const Properties& properties)
    : RollingJointBase(properties)
{
    mJacobianDeriv = Eigen::Vector6d::Zero();

    // Inherited Aspects must be created in the final joint class in reverse order
    // or else we get pure virtual function calls
    createRollingJointAspect(properties);
    createGenericJointAspect(properties);
    createJointAspect(properties);
}

Joint* RollingJoint::clone() const
{
    return new RollingJoint(getRJProperties());
}



void RollingJoint::updateDegreeOfFreedomNames()
{
    // Same name as the joint it belongs to.
    if(!mDofs[0]->isNamePreserved())
        mDofs[0]->setName(Joint::mAspectProperties.mName, false);
}



void RollingJoint::updateRelativeTransform() const
{
    mT = Joint::mAspectProperties.mT_ParentBodyToJoint
        * Eigen::Translation3d(Eigen::AngleAxisd( getPositionsStatic()(0), mAspectProperties.m_axis ) * mAspectProperties.m_j1_axis_offset)
        * math::expAngular(mAspectProperties.m_axis * getPositionsStatic()(0) * (1+mAspectProperties.m_r0_r1_ratio) )
        * Joint::mAspectProperties.mT_ChildBodyToJoint.inverse();

    // Verification
    assert(math::verifyTransform(mT));
}





//TODO: Check the below functions
void RollingJoint::updateRelativeJacobian(bool mandatory) const
{
    mJacobian = getRelativeJacobianStatic(getPositionsStatic());
}

GenericJoint<math::R1Space>::JacobianMatrix
RollingJoint::getRelativeJacobianStatic(
        const GenericJoint<math::R1Space>::Vector& pos) const
{
    // Revolute example:
    // GenericJoint<math::R1Space>::JacobianMatrix J =
    //   math::AdTAngular(Joint::mAspectProperties.mT_ChildBodyToJoint, getAxis());



    // Eigen::Vector6d joint_inputSVjoint_output;
    // joint_inputSVjoint_output << mAspectProperties.m_axis * (1+mAspectProperties.m_r0_r1_ratio) , 
    //   mT.linear().transpose() * mAspectProperties.m_axis.cross( Eigen::AngleAxisd( getPositionsStatic()(0), mAspectProperties.m_axis ) * mAspectProperties.m_j1_axis_offset );

    // GenericJoint<math::R1Space>::JacobianMatrix J =
    //   math::AdT(Joint::mAspectProperties.mT_ChildBodyToJoint, joint_inputSVjoint_output);


    // -- OR --


    GenericJoint<math::R1Space>::JacobianMatrix J =
        math::AdTAngular(Joint::mAspectProperties.mT_ChildBodyToJoint, mAspectProperties.m_axis * (1+mAspectProperties.m_r0_r1_ratio));

    J.tail<3>() = mT.linear().transpose() * mAspectProperties.m_axis.cross( Eigen::AngleAxisd( getPositionsStatic()(0), mAspectProperties.m_axis ) * mAspectProperties.m_j1_axis_offset );
    // J.tail<3>() = math::AdTLinear( Joint::mAspectProperties.mT_ChildBodyToJoint , mT.linear().transpose() 
    //    * mAspectProperties.m_axis.cross( Eigen::AngleAxisd( getPositionsStatic()(0), mAspectProperties.m_axis ) * mAspectProperties.m_j1_axis_offset ) ).tail<3>();


    assert(!math::isNan(J));
    return J;
}


void RollingJoint::updateRelativeJacobianTimeDeriv() const
{
    assert(mJacobianDeriv == Eigen::Vector6d::Zero());
}


//==============================================================================
const Eigen::Vector3d& RollingJoint::getAxis() const
{
    return mAspectProperties.m_axis;
}
