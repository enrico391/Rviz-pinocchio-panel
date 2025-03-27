// Description: Class to manage the Pinocchio model

#include <rviz_panel_pinocchio_tiago/pinocchio_panel_manage.hpp>

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

    
    int cont = 0;
    // start from 1 to skip universe joint
    for(int i = 1; i <= (int)q_conf.size(); ++i){
        std::cout <<" i: " << i << " cont : " << cont << model.names[i] <<std::endl;
        // for continuous joints (e.g wheels)
        if (model.joints[i].nq() == 2){
            q[cont] = cos(q_conf[i-1]);
            q[cont+1] = sin(q_conf[i-1]);
        } 
        // normal joint type revolute or prismatic
        else {
            q[cont] = q_conf[i-1];
        }
        cont += (int)model.joints[i].nq();
        
        
    }

    std::cout << "Configuration vector q: " << q.transpose() << std::endl;
    //q << 0.0, 0.0, 0.0, 1.36, -0.0, -0.0, 2.22, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0.0, 0.0;
    std::cout << "Configuration done " << std::endl;
}


std::vector<std::string>  PinocchioManager::getConfiguration() {
    std::vector<std::string> frame_names;
    // get the names of the frames and add to the vector of strings
    for (const auto &frame : model.frames) {
        frame_names.push_back(frame.name);
    }

    for (size_t i = 1; i < model.joints.size(); ++i) { // Joint index starts from 1
        if (model.joints[i].nq() > 0) {  // Movable joints have nq > 0
            std::cout << "Joint " << i << ": " << model.names[i] << " (DOFs: " << model.joints[i].nq() << ")" << std::endl;
        }
    }

    return frame_names; 
}


std::vector<std::string>  PinocchioManager::getActiveJointsName() {
    std::vector<std::string> joints_names;
    
    // Filter and add only active joint names to the vector of strings
    for (size_t i = 1; i < model.joints.size(); ++i) { // Joint index starts from 1
        if (model.joints[i].nq() > 0) {  // Movable joints have nq > 0
            joints_names.push_back(model.names[i]);
        }
    }
    
    return joints_names; 
}


void PinocchioManager::performForwardKinematics() {
    pinocchio::framesForwardKinematics(model, data, q);
}

//TODO
std::vector<double> PinocchioManager::performTorqueEstimation() {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv); // in rad/s 
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv); // in rad/s² 
    Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);
    
    std::vector<double> joint_torques(data.tau.size());
    for(int i = 0; i < (int)data.tau.size(); i++){
        joint_torques[i] = data.tau[i];
    }

    return joint_torques;
}


std::string PinocchioManager::getFrameTransform(const std::string& arm_link_name) {
    const auto arm_link = model.getFrameId(arm_link_name);
    //std::cout << "Frame transform: " << std::endl << data.oMf[arm_link] << std::endl;
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
        //std::cout << "Collision detected between " << body1 << " and " << body2 << std::endl;
      }
    }

    return collision_pairs;
}

} // namespace rviz_panel_pinocchio_tiago