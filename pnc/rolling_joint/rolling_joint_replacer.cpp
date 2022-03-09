#include "pnc/rolling_joint/rolling_joint_replacer.hpp"

#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/WeldJoint.hpp>

bool RollingJointReplacer::handles(dart::dynamics::BodyNode* node)
{
    std::vector<std::string> name_tokens = tokenizePrefixes(node->getParentJoint()->getName());
    std::size_t num_tokens = name_tokens.size();

    std::string joint_name = name_tokens.at(0);
    if(num_tokens >= 2)
    {
        std::string joint_type = name_tokens.at(1);
        if( boost::iequals(joint_type, "rolling") || boost::iequals(joint_type, "roll") )
        {
            if(num_tokens >= 3)
            {
                std::string joint_num = name_tokens.at(2);
                if( boost::iequals(joint_num, "j0") || boost::iequals(joint_num, "j1") )
                {
                    return true;
                }
            }
        }
    }
    return false;
}

dart::dynamics::BodyNode* RollingJointReplacer::replace(dart::dynamics::SkeletonPtr old_skeleton, dart::dynamics::SkeletonPtr new_skeleton, dart::dynamics::BodyNode* old_node, dart::dynamics::BodyNode* new_parent)
{
    dart::dynamics::BodyNode* next_new_parent = 0;

    if(this->handles(old_node))
    {
        dart::dynamics::Joint* joint = old_node->getParentJoint();
        std::vector<std::string> name_tokens = tokenizePrefixes(joint->getName());
        std::size_t num_tokens = name_tokens.size();
        std::string joint_name = name_tokens.at(0);
        std::string joint_type = name_tokens.at(1);
        std::string joint_num = name_tokens.at(2);

        if(boost::iequals(joint_num, "j0"))
        {

            // return replaceL1withFixed(old_skeleton, new_skeleton, old_node, new_parent);

            //  -- OR --

            m_mimic_joint_map[joint_name] = old_node->copyTo(new_skeleton, new_parent, false).first;
            m_mimic_joint_map[joint_name]->setActuatorType(dart::dynamics::Joint::MIMIC);
            return new_parent;
        }

        // else if( boost::iequals(joint_num, "j1") )


        double r0_r1_ratio = 1.0;

        //if(!m_nh.hasParam("/cortex_framework/rolling_joint_ratio/" + joint_name))
            //std::cerr << "WARNING: Rolling joint replacement plugin could not find parameter '/cortex_framework/rolling_joint_ratio/" << joint_name << "'.  Setting this ratio equal to '1'." << std::endl;
        //else
            //m_nh.param("/cortex_framework/rolling_joint_ratio/" + joint_name, r0_r1_ratio, r0_r1_ratio);


        dart::dynamics::RevoluteJoint* rev_joint = (dart::dynamics::RevoluteJoint*) joint;
        Eigen::Vector3d axis = rev_joint->getAxis();
        Eigen::Vector3d j1_axis_offset = joint->getTransformFromParentBodyNode().translation();


        std::cerr << "RollingJointReplacer params: " << std::endl;
        std::cerr << " -- axis:           " << axis.transpose() << std::endl;
        std::cerr << " -- j1_axis_offset: " << j1_axis_offset.transpose() << std::endl;
        std::cerr << " -- r0_r1_ratio:    " << r0_r1_ratio << std::endl;


        RJUniqueProperties rjup = RJUniqueProperties(axis, j1_axis_offset, r0_r1_ratio);
        RollingJoint::Properties properties( rev_joint->getGenericJointProperties(), rjup);

        std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> new_joint_body = 
            new_skeleton->createJointAndBodyNodePair<RollingJoint, dart::dynamics::BodyNode>(new_parent, properties, old_node->getBodyNodeProperties());

        // RollingJoint* rj = (RollingJoint*) new_joint_body.first;

        new_joint_body.first->setTransformFromParentBodyNode(joint->getParentBodyNode()->getParentJoint()->getTransformFromParentBodyNode());
        // rj->properties->mT_ParentBodyToJoint *= old_node->getParentBodyNode()->properties->mT_ParentBodyToJoint;


        //copy shapes
        for(unsigned int i = 0; i < old_node->getNumShapeNodes(); i++)
        {
            dart::dynamics::ShapeNode* shapeNode = new_joint_body.second->createShapeNode(dart::dynamics::ShapeNode::BasicProperties(), true);
            shapeNode->duplicateAspects(old_node->getShapeNode(i));
            shapeNode->copy(old_node->getShapeNode(i));
        }

        // set up the mimic joint properly
        m_mimic_joint_map.at(joint_name)->setMimicJoint(new_joint_body.first);

        std::string dof_name = m_mimic_joint_map.at(joint_name)->getDof(0)->getName();
        m_mimic_joint_map.at(joint_name)->set< dart::dynamics::GenericJoint< dart::math::RealVectorSpace<1> >::Aspect >( 
                rev_joint->get< dart::dynamics::GenericJoint< dart::math::RealVectorSpace<1> >::Aspect >()
                ); // new rolling joint has same GenerictJointAspects as old revolute
        m_mimic_joint_map.at(joint_name)->getDof(0)->setName(dof_name);


        return new_joint_body.second;
    }

    return next_new_parent;
}


dart::dynamics::BodyNode* RollingJointReplacer::replaceL1withFixed(dart::dynamics::SkeletonPtr old_skeleton, dart::dynamics::SkeletonPtr new_skeleton, dart::dynamics::BodyNode* old_node, dart::dynamics::BodyNode* new_parent)
{
    std::cerr << "Replacing j0 with fixed joint ..." << std::endl;
    dart::dynamics::BodyNode* next_new_parent=0;

    // Grab the joint we're changing
    dart::dynamics::Joint* parent_joint = old_node->getParentJoint();
    double adjustment_value = 0.0;

    if(parent_joint->getType() != "RevoluteJoint")
    {
        std::cerr << "Error: Rolling joint replacement plugin detected invalid joint type - " << parent_joint->getType() << " for joint " << parent_joint->getName() << std::endl;
        return 0;
    }

    std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> new_joint_body = 
        new_skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint, dart::dynamics::BodyNode>(
                new_parent, parent_joint->getJointProperties(), old_node->getBodyNodeProperties());

    next_new_parent = new_parent;                 // in this case we'll use seperate chain for fixed joint

    // Fix dart parameters? and change name
    // std::vector<std::string> name_tokens = aptk::comm::tokenizePrefixes(parent_joint->getName());
    // new_joint_body.first->setName(parent_joint->getName());


    //copy shapes
    for(unsigned int i = 0; i < old_node->getNumShapeNodes(); i++)
    {
        dart::dynamics::ShapeNode* shapeNode = new_joint_body.second->createShapeNode(dart::dynamics::ShapeNode::BasicProperties(), true);
        shapeNode->duplicateAspects(old_node->getShapeNode(i));
        shapeNode->copy(old_node->getShapeNode(i));
    }

    return next_new_parent;
}

