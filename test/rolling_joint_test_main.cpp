#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include "test/rolling_joint_test_world_node.hpp"
#include "util/util.hpp"
#include "configuration.hpp"

#include "pnc/rolling_joint/rolling_joint_replacer.hpp"

//inline std::vector<std::string> tokenizePrefixes(std::string name){
      //boost::replace_all(name, "__", ",");
      //std::vector<std::string> tokens;

      //boost::char_separator<char> sep {","};
      //boost::tokenizer<boost::char_separator<char> > tokenizer(name, sep);
      //for(boost::tokenizer<boost::char_separator<char> >::iterator it = tokenizer.begin(); it != tokenizer.end(); ++it)
      //{
        //std::string token(*it);
        //tokens.push_back(token);
      //}

      //return tokens;
//}

void displayJointFrames(const dart::simulation::WorldPtr &world,
                        const dart::dynamics::SkeletonPtr &robot) {
  for (std::size_t i = 0; i < robot->getNumBodyNodes(); ++i) {
    dart::dynamics::BodyNode *bn = robot->getBodyNode(i);
    for (std::size_t j = 0; j < bn->getNumChildJoints(); ++j) {
      const dart::dynamics::Joint *joint = bn->getChildJoint(j);
      const Eigen::Isometry3d offset = joint->getTransformFromParentBodyNode();

      dart::gui::osg::InteractiveFramePtr frame =
          std::make_shared<dart::gui::osg::InteractiveFrame>(
              bn, joint->getName() + "/frame", offset);

      for (const auto type : {dart::gui::osg::InteractiveTool::ANGULAR,
                              dart::gui::osg::InteractiveTool::PLANAR})
        for (std::size_t i = 0; i < 3; ++i)
          frame->getTool(type, i)->setEnabled(false);

      world->addSimpleFrame(frame);
    }
  }
}


class OneStepProgress : public osgGA::GUIEventHandler {
public:
  OneStepProgress(RollingJointWorldNode *worldnode) : worldnode_(worldnode) {}

  /** Deprecated, Handle events, return true if handled, false otherwise. */
  virtual bool handle(const osgGA::GUIEventAdapter &ea,
                      osgGA::GUIActionAdapter & /*aa*/) {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYUP) {
      // custom buttons
      // World Node Buttons
      if (ea.getKey() == 'p') {
      }
      if (ea.getKey() == 'r') {
      }
      if(ea.getKey() == 'w') {
      }
      if (ea.getKey() == 'a') {
      }
      if (ea.getKey() == 's') {
      }
      if (ea.getKey() == 'd') {
      }
      if (ea.getKey() == 'q') {
      }
      if (ea.getKey() == 'e') {
      }
      if (ea.getKey() == 'x') {
      }
      if (ea.getKey() == 'j') {
      }
      if (ea.getKey() == 'k') {
      }

      if (ea.getKey() == 'f') {
        int numStepProgress(50);
        for (int i = 0; i < numStepProgress; ++i) {
          worldnode_->customPreStep();
          worldnode_->getWorld()->step();
          worldnode_->customPostStep();
        }
        return true;
      }
    }
    return false;
  }
  RollingJointWorldNode *worldnode_;
};

// void _setJointLimitConstraint(dart::dynamics::SkeletonPtr robot) {
// for (int i = 0; i < robot->getNumJoints(); ++i) {
// dart::dynamics::Joint *joint = robot->getJoint(i);
// joint->setPositionLimitEnforced(true);
//}
//}

void _printRobotModel(dart::dynamics::SkeletonPtr robot) {
   for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
   dart::dynamics::BodyNodePtr bn = robot->getBodyNode(i);
   std::cout << i << "th" << std::endl;
   std::cout << bn->getName() << std::endl;
   std::cout << bn->getMass() << std::endl;
  }

   for (int i = 0; i < robot->getNumJoints(); ++i) {
   dart::dynamics::Joint* joint = robot->getJoint(i);
   std::cout << i << "th" << std::endl;
   std::cout << joint->getNumDofs() << std::endl;
  }

  for (int i = 0; i < robot->getNumDofs(); ++i) {
    dart::dynamics::DegreeOfFreedom *dof = robot->getDof(i);
    std::cout << i << "th" << std::endl;
    std::cout << "dof name : " << dof->getName() << std::endl;
     std::cout << "child body node name and mass : "
    << dof->getChildBodyNode()->getName() << " , "
    << dof->getChildBodyNode()->getMass() << std::endl;
  }

   std::cout << "num dof" << std::endl;
   std::cout << robot->getNumDofs() << std::endl;
   std::cout << robot->getNumJoints() << std::endl;
   std::cout << "mass mat row" << std::endl;
   std::cout << robot->getMassMatrix().rows() << std::endl;
   std::cout << robot->getMassMatrix().cols() << std::endl;
   std::cout << "q" << std::endl;
   std::cout << robot->getPositions() << std::endl;
   std::cout << "robot total mass" << std::endl;
   std::cout << robot->getMass() << std::endl;
   std::cout << "robot position" << std::endl;
   std::cout << robot->getPositions() << std::endl;

   std::cout << "==================" << std::endl;

   //std::cout << "right" << std::endl;
   //std::cout << robot->getBodyNode("rightCOP_Frame")
  //->getWorldTransform()
  //.translation()
  //<< std::endl;
   //std::cout << "left" << std::endl;
   //std::cout << robot->getBodyNode("leftCOP_Frame")
  //->getWorldTransform()
  //.translation()
  //<< std::endl;

  //exit(0);
}

void SetInitialConfiguration(dart::dynamics::SkeletonPtr robot) {
  //int j0_idx = robot->getDof("j0")->getIndexInSkeleton();
  int j1_idx = robot->getDof("joint_name__roll__j1")->getIndexInSkeleton();

  Eigen::Isometry3d fixed_base_joint_iso = Eigen::Isometry3d::Identity();

  Eigen::Vector3d fixed_base_joint_pos = Eigen::Vector3d::Zero();
  fixed_base_joint_pos << 0, 0, 0;

  fixed_base_joint_iso.translation() = fixed_base_joint_pos;

  robot->getJoint(0)->setTransformFromParentBodyNode(fixed_base_joint_iso);


  Eigen::VectorXd q = robot->getPositions();
  //q[j0_idx] = M_PI / 4.;
  q[j1_idx] = M_PI / 4.;

  robot->setPositions(q);
}

//void LocateReplacementRecursive(dart::dynamics::BodyNode* node, RollingJointReplacer *replacer){

    //if(replacer->handles(node))
//}

void ModifySkeletonRecursive(dart::dynamics::SkeletonPtr old_skel, 
                        dart::dynamics::SkeletonPtr new_skel, 
                        dart::dynamics::BodyNode *old_node, 
                        dart::dynamics::BodyNode *new_parent,
                        RollingJointReplacer *replacer){

std::cout << "old_node name: " << old_node->getName() << std::endl;
dart::dynamics::BodyNode *next_new_parent = nullptr;

next_new_parent = replacer->replace(old_skel, new_skel, old_node, new_parent);

if(next_new_parent == nullptr){
    next_new_parent = old_node->copyTo(new_skel, new_parent, false).second;
}

for(unsigned int i = 0; i < old_node->getNumChildBodyNodes(); i++){
    ModifySkeletonRecursive(old_skel, new_skel, old_node->getChildBodyNode(i), next_new_parent, replacer);
}

}

int main(int argc, char **argv) {
  double servo_dt;
  bool isRecord;
  bool b_show_joint_frame;
  try {
    YAML::Node simulation_cfg =
        YAML::LoadFile(THIS_COM "config/manipulator/dart_simulation.yaml");
    util::ReadParameter(simulation_cfg, "servo_dt", servo_dt);
    util::ReadParameter(simulation_cfg, "is_record", isRecord);
    util::ReadParameter(simulation_cfg, "show_joint_frame", b_show_joint_frame);
  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
  }
  // =========================================================================
  // Generate world and add skeletons
  // =========================================================================
  dart::simulation::WorldPtr world(new dart::simulation::World);
  dart::utils::DartLoader urdfLoader;
  dart::dynamics::SkeletonPtr ground = urdfLoader.parseSkeleton(
      THIS_COM "robot_model/ground/ground_terrain.urdf");
  dart::dynamics::SkeletonPtr robot_model_raw = urdfLoader.parseSkeleton(
      THIS_COM "robot_model/manipulator/rolling_joint_model.urdf");


  //TODO: modify skeleton
  dart::dynamics::SkeletonPtr robot = dart::dynamics::Skeleton::create(robot_model_raw->getName());//create empty skeleton
  robot_model_raw->setName(robot_model_raw->getName()+"_raw");
  robot_model_raw->setPositions(Eigen::VectorXd::Zero(robot_model_raw->getNumDofs()));
  robot_model_raw->setVelocities(Eigen::VectorXd::Zero(robot_model_raw->getNumDofs()));
  robot_model_raw->computeForwardKinematics();

  RollingJointReplacer *replacer = new RollingJointReplacer();

  //locate replacement recursively
  //LocateReplacementRecursive(robot_model_raw->getRootBodyNode(), replacer);
  //
  //modify skeleton recursively
  ModifySkeletonRecursive(robot_model_raw, robot, robot_model_raw->getRootBodyNode(), 0, replacer);




  world->addSkeleton(ground);
  world->addSkeleton(robot);

  // =========================================================================
  // Friction & Restitution Coefficient
  // =========================================================================
  // double friction(0.5);
  // double restit(0.0);
  // ground->getBodyNode("planeLink")->setFrictionCoeff(friction);
  // robot->getBodyNode("l_foot")->setFrictionCoeff(friction);
  // robot->getBodyNode("r_foot")->setFrictionCoeff(friction);

  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  world->setGravity(gravity);
  world->setTimeStep(servo_dt);

  // =========================================================================
  // Display Joints Frame
  // =========================================================================
  if (b_show_joint_frame)
    displayJointFrames(world, robot);

  // =========================================================================
  // Initial configuration
  // =========================================================================
  SetInitialConfiguration(robot);

  // =========================================================================
  // Enabel Joit Limits
  // =========================================================================
  //_setJointLimitConstraint(robot);

  // =========================================================================
  // Print Model Info
  // =========================================================================
  _printRobotModel(robot);
  _printRobotModel(robot_model_raw);

  // =========================================================================
  // Wrap a worldnode
  // =========================================================================
  osg::ref_ptr<RollingJointWorldNode> node;
  node = new RollingJointWorldNode(world);
  node->setNumStepsPerCycle(30);

  // =========================================================================
  // Create and Set Viewer
  // =========================================================================
  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.simulate(false);
  viewer.switchHeadlights(false);
  ::osg::Vec3 p1(1.0, 0.2, 1.0);
  p1 = p1 * 0.7;
  viewer.getLightSource(0)->getLight()->setPosition(
      ::osg::Vec4(p1[0], p1[1], p1[2], 0.0));
  viewer.getCamera()->setClearColor(osg::Vec4(0.93f, 0.95f, 1.0f, 0.95f));
  viewer.getCamera()->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  viewer.addEventHandler(new OneStepProgress(node));

  if (isRecord) {
    std::cout << "[Video Record Enable]" << std::endl;
    viewer.record(THIS_COM "/video");
  }

  // viewer.setUpViewInWindow(0, 0, 2880, 1800);
  viewer.setUpViewInWindow(1440, 0, 500, 500);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.14, 2.28, 3.0) * 1.5, ::osg::Vec3(0.0, 0.2, 0.5),
      ::osg::Vec3(0.0, 0.0, 1.0));
  viewer.setCameraManipulator(viewer.getCameraManipulator());
  viewer.run();
}
