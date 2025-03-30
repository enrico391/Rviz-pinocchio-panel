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


    // initialize backup models
    model_backup = model;
    data_backup = data;
    visual_model_backup = visual_model;
    collision_data_backup = collision_data;
}

void PinocchioManager::setConfiguration(const std::vector<double>& q_conf) {
    std::cout << "Joint configuration: " << std::endl << model.nq << std::endl << std::endl;
    
    Eigen::VectorXd config(model.nq);
    q = config;

    
    int cont = 0;
    // start from 1 to skip universe joint
    for(int i = 1; i <= (int)q_conf.size(); ++i){
        // std::cout <<" i: " << i << " cont : " << cont << model.names[i] <<std::endl;
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



void PinocchioManager::addPayloadMass(const double& mass) {
    Eigen::Vector3d payload_com(0.0, 0.0, 0.1);  // Payload CoM relative to the attachment point
    double sphere_radius = 0.05;  // Approximate shape of payload
    
    // TODO REMOVE ALL VALUE OF MASS IN ORDER TO NOT ADD MULTIPLE MASSES EVERY TIME I WANT TO ADD A MASS (POSSIBLE SOLUTION: ADD A BACKUP MODEL)
    if(massAdded){
        model = model_backup;
        data = data_backup;
        visual_model = visual_model_backup;
        collision_data = collision_data_backup;
        collision_model = collision_model_backup;
        massAdded = false;
    }

    // Compute payload inertia assuming it is a mass concentrated in a point
    pinocchio::Inertia payload_inertia = pinocchio::Inertia::FromSphere(mass, sphere_radius);
    
    // Get the parent joint ID 
    pinocchio::JointIndex parent_frame_id = model.getFrameId("gripper_left_finger_joint");
    
    if (parent_frame_id >= model.frames.size()) {
        std::cout << "Parent frame not found" << std::endl;
        return;
    }
    
    // Get the parent joint ID from the parent frame
    pinocchio::JointIndex parent_joint_id = model.frames[parent_frame_id].parent;
    
    // Create transformation from parent joint to payload
    pinocchio::SE3 payload_transform = pinocchio::SE3::Identity();
    payload_transform.translation() = payload_com;
    
    // Add the payload body directly to the parent joint (fixed joint approach)
    model.appendBodyToJoint(parent_joint_id, payload_inertia, payload_transform);
    
    // Update the data structures after modifying the model
    data = pinocchio::Data(model);

    massAdded = true;
    std::cout << "Payload mass added: " << mass << " kg" << std::endl;
}


void checkMassAdded(){

}




} // namespace rviz_panel_pinocchio_tiago