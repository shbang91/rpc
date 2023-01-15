#pragma once
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/GenericJoint.hpp>
#include <dart/math/Geometry.hpp>
#include <dart/math/Helpers.hpp>

#include <string>

using DartGeneric1DJoint = dart::dynamics::GenericJoint<dart::math::R1Space>;
struct RJUniqueProperties {
  RJUniqueProperties(
      const Eigen::Vector3d &axis = Eigen::Vector3d::UnitZ(),
      const Eigen::Vector3d &j1_axis_offset = Eigen::Vector3d::UnitX(),
      const double &r0_r1_ratio = 1.0);

  virtual ~RJUniqueProperties() = default;

  // axis of rotation of j0 and j1
  Eigen::Vector3d m_axis;

  // translational offset from j0 to j1 at the zero position
  // should be unit vector, and should lie in plane perpendicular to m_axis
  Eigen::Vector3d m_j1_axis_offset;

  // ratio of two radii, r0/r1
  double m_r0_r1_ratio;
};

struct RJProperties : DartGeneric1DJoint::Properties, RJUniqueProperties {
  DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(RJProperties)

  RJProperties(const DartGeneric1DJoint::Properties &genericJointProperties =
                   DartGeneric1DJoint::Properties(),
               const RJUniqueProperties &plProperties = RJUniqueProperties());

  virtual ~RJProperties() = default;
};

class RollingJoint;
using RollingJointBase =
    dart::common::EmbedPropertiesOnTopOf<RollingJoint, RJUniqueProperties,
                                         DartGeneric1DJoint>;

class RollingJoint : public RollingJointBase {
public:
  friend class dart::dynamics::Skeleton;
  using UniqueProperties = RJUniqueProperties;
  using Properties = RJProperties;
  using Base = RollingJointBase;
  DART_BAKE_SPECIALIZED_ASPECT_IRREGULAR(Aspect, RollingJointAspect)

  RollingJoint(const RollingJoint &) = delete;
  virtual ~RollingJoint();

  const Eigen::Vector3d &getAxis() const;

  // will need most below this
  void setProperties(const Properties &_properties);
  void setProperties(const UniqueProperties &_properties);
  void setAspectProperties(const AspectProperties &properties);
  Properties getRJProperties() const;

  void copy(const RollingJoint &_otherJoint);
  void copy(const RollingJoint *_otherJoint);
  RollingJoint &operator=(const RollingJoint &_otherJoint);

  const std::string &getType() const override;
  static const std::string &getStaticType();
  bool isCyclic(std::size_t _index) const override;

  // dont think this is needed
  // void setPosLimits(double min, double max);

  // this is necessary
  DartGeneric1DJoint::JacobianMatrix getRelativeJacobianStatic(
      const DartGeneric1DJoint::Vector &positions) const override;

protected:
  RollingJoint(const Properties &properties);
  Joint *clone() const override;
  void updateDegreeOfFreedomNames() override;
  // need these three here
  void updateRelativeTransform() const override;
  void updateRelativeJacobian(bool mandatory = true) const override;
  void updateRelativeJacobianTimeDeriv() const override;

  // ??
  // void nonStupidUpdateRelativeTransform();
};
