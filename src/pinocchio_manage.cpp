// Description: Class to manage the Pinocchio model

#include <rviz_panel_pinocchio_tiago/demo_panel.hpp>

#define PINOCCHIO_ENABLE_COMPATIBILITY_WITH_VERSION_2


namespace rviz_panel_pinocchio_tiago
{
// Class to manage the Pinocchio model 
PinocchioManager::PinocchioManager(const std::string& urdf_xml, const std::string& package_share_directory) {
    
    //build the model from the URDF
    pinocchio::urdf::buildModelFromXML(urdf_xml, model);
    pinocchio::urdf::buildGeom(model, std::istringstream(urdf_xml), pinocchio::VISUAL, visual_model);
    pinocchio::urdf::buildGeom(model, std::istringstream(urdf_xml), pinocchio::COLLISION, collision_model);

    // add all collision pairs from srdf file
    collision_model.addAllCollisionPairs();
    pinocchio::srdf::removeCollisionPairs(model, collision_model, package_share_directory + "/srdf/tiago.srdf", false);

    // create data model
    pinocchio::Data data_model(model);
    data = data_model;
    
    // create collision data model
    pinocchio::GeometryData collision_data_model(collision_model);
    collision_data = collision_data_model;
}

void PinocchioManager::setConfiguration(const std::vector<double> q_conf) {
    std::cout << "Joint configuration: " << std::endl << model.nq << std::endl << std::endl;
    Eigen::VectorXd config(model.nq);
    q = config;

    //TODO fix error number of joint 14 instead of 16 
    
    for(int i = 0; i < (int)q.size(); i++){
        if (i == 0){
            q[i] = 0.0; //for first 
        } 
        else if (i == (model.nq - 1)) {
            q[i] = 0.0; // for last
        } else {
            q[i] = q_conf[i-1];
        }
        
    }

    std::cout << "Configuration vector q: " << q.transpose() << std::endl;
    //q << 0.0, 0.0, 0.0, 1.36, -0.0, -0.0, 2.22, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0;
    std::cout << "Configuration done " << std::endl;
}


std::vector<std::string>  PinocchioManager::getConfiguration() {
    std::vector<std::string> frame_names;
    for (int joint_id = 0; joint_id < model.njoints; joint_id++) {
        frame_names.push_back(model.names[joint_id]);
    }

    return frame_names; 
}


void PinocchioManager::performForwardKinematics() {
    pinocchio::framesForwardKinematics(model, data, q);
}


std::string PinocchioManager::getFrameTransform(const std::string& arm_link_name) {
    const auto arm_link = model.getFrameId(arm_link_name);
    std::cout << "Frame transform: " << std::endl << data.oMf[arm_link] << std::endl;
    std::ostringstream oss;
    oss << data.oMf[arm_link];
    return oss.str();
}


void PinocchioManager::performJacobian(const std::string& arm_link) {
    Eigen::MatrixXd ee_jacobian(6, model.nv);
    pinocchio::computeFrameJacobian(model, data, q, model.getFrameId(arm_link), ee_jacobian);
}


std::vector<std::string> PinocchioManager::performCollisionCheck() {
    pinocchio::computeCollisions(model, data, collision_model, collision_data, q);
    
    std::vector<std::string> collision_pairs;

    for (size_t k = 0; k < collision_model.collisionPairs.size(); ++k)
    {
      const auto& cp = collision_model.collisionPairs[k];
      const auto& cr = collision_data.collisionResults[k];
      if (cr.isCollision())
      {
        const auto& body1 = collision_model.geometryObjects[cp.first].name;
        const auto& body2 = collision_model.geometryObjects[cp.second].name;
        
        collision_pairs.push_back("Collision detected between " + body1 + " and " + body2);
        std::cout << "Collision detected between " << body1 << " and " << body2 << std::endl;
      }
    }

    return collision_pairs;
}

} // namespace rviz_panel_pinocchio_tiago