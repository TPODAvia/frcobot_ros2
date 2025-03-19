#include "task_builder.hpp"

namespace fs = std::filesystem;

namespace {
// --- Helper structures and functions for G-code parsing ---
struct GCodeCommand
{
    std::string original_line;
    std::string code;
    std::map<char, double> params;
    std::string comment;
};

std::string extractCommandToken(const std::string& token)
{
    if (token.empty())
        return "";
    char c = std::toupper(token[0]);
    if (c == 'G' || c == 'M' || c == 'T')
        return token;
    return "";
}

GCodeCommand parseGCodeLine(const std::string& raw_line)
{
    GCodeCommand cmd;
    cmd.original_line = raw_line;
    std::string line = raw_line;
    auto sc_pos = line.find(';');
    if (sc_pos != std::string::npos)
    {
        cmd.comment = line.substr(sc_pos + 1);
        line = line.substr(0, sc_pos);
    }
    auto openParen = line.find('(');
    auto closeParen = line.find(')', openParen + 1);
    if (openParen != std::string::npos && closeParen != std::string::npos)
    {
        std::string parenComment = line.substr(openParen + 1, closeParen - openParen - 1);
        if (!cmd.comment.empty())
            cmd.comment += " | ";
        cmd.comment += parenComment;
        line.erase(openParen, closeParen - openParen + 1);
    }
    auto trimSpace = [](std::string & s)
    {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            [](unsigned char ch){ return !std::isspace(ch); }));
        s.erase(std::find_if(s.rbegin(), s.rend(),
            [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end());
    };
    trimSpace(line);
    std::stringstream ss(line);
    std::string token;
    bool firstTokenProcessed = false;
    while (ss >> token)
    {
        if (!firstTokenProcessed)
        {
            std::string possibleCmd = extractCommandToken(token);
            if (!possibleCmd.empty())
            {
                cmd.code = possibleCmd;
                firstTokenProcessed = true;
                continue;
            }
            firstTokenProcessed = true;
        }
        if (token.size() >= 2)
        {
            char paramLetter = std::toupper(token[0]);
            std::string valStr = token.substr(1);
            try
            {
                double value = std::stod(valStr);
                cmd.params[paramLetter] = value;
            }
            catch(...) { /* ignore non-numeric tokens */ }
        }
    }
    return cmd;
}
} // end anonymous namespace


TaskBuilder::TaskBuilder(rclcpp::Node::SharedPtr node,
                         const std::string& arm_group_name,
                         const std::string& tip_frame)
    : node_{node}
    , arm_group_name_{arm_group_name}
    , tip_frame_{tip_frame}
{
    // Initialize solvers
    sampling_planner_  = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();

    // Set default solver
    current_solver_ = sampling_planner_;

    // Initialize default solvers map
    solvers_["sampling_planner"] = sampling_planner_;

    // Initialize move_group_ (using MoveIt2’s MoveGroupInterface)
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_name_);

    // Use ament_index_cpp to get the path to your ROS 2 package's share dir
    std::string package_share_directory;
    try {
        package_share_directory = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to get package share directory for 'fairino_mtc_demo': %s",
                     e.what());
        package_share_directory = ".";
    }

    // Memory directory
    std::string memory_dir = package_share_directory + "/memory/";
    // YAML config file
    std::string config_file = memory_dir + "current_solver_config.yaml";

    // Attempt to load solver config
    if (loadSolverConfig(config_file)) {
        RCLCPP_INFO(node_->get_logger(), "Loaded solver configuration from %s", config_file.c_str());
    } else {
        RCLCPP_WARN(node_->get_logger(), "Failed to load solver configuration. Using default solver.");
    }

    // Use Reliable QoS profile for joint states subscription
    rclcpp::QoS qos_profile = rclcpp::QoS(2).reliable();
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        qos_profile,
        std::bind(&TaskBuilder::jointStateCallback, this, std::placeholders::_1));

    executed_ = true;
}

void TaskBuilder::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (joint_names_.empty() && !msg->name.empty()) {
        joint_names_ = msg->name;  // Store joint names from the first message
    }

    // Store joint positions in the map
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (i < msg->position.size()) {
            current_joint_positions_[msg->name[i]] = msg->position[i];
        }
    }
    executed_ = true;
}

void TaskBuilder::newTask(const std::string& task_name)
{
    // Clear any existing task
    task_.clear();
    // Create a brand new MTC Task
    task_.stages()->setName(task_name);
    task_.loadRobotModel(node_);

    // Set MTC properties from our member variables
    task_.setProperty("group", arm_group_name_);
    task_.setProperty("ik_frame", tip_frame_);

    // Always start with a CurrentState stage as the first stage
    auto current_state_stage = std::make_unique<mtc::stages::CurrentState>("current state");
    task_.add(std::move(current_state_stage));
    executed_ = true;
}

void TaskBuilder::savePipelineConfig(const std::string& pipeline_name,
                                     const std::string& planner_id,
                                     double max_vel_factor,
                                     double max_acc_factor,
                                     double tolerance)
{
    std::string package_share_directory;
    try {
        package_share_directory = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to get package share directory for 'fairino_mtc_demo': %s",
                     e.what());
        executed_ = false;
        return;
    }

    const std::string memory_dir = package_share_directory + "/memory/";
    if (!fs::exists(memory_dir)) {
        try {
            fs::create_directories(memory_dir);
            RCLCPP_INFO(node_->get_logger(), "Created memory directory at %s", memory_dir.c_str());
        } catch (const fs::filesystem_error& e) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Failed to create memory directory: %s",
                         e.what());
            executed_ = false;
            return;
        }
    }

    YAML::Node config;
    config["pipeline_name"] = pipeline_name;
    config["planner_id"] = planner_id;

    // We’ll use local variables to hold the defaults,
    // then override them only if the user did NOT pass in NaN.
    double temp_vel = 0.0;
    double temp_acc = 0.0;
    double temp_tol = 0.0;  // default fallback

    // Pipeline-based defaults (the old forced assignments):
    if (pipeline_name == "pilz_industrial_motion_planner")
    {
        if (planner_id == "PTP")
        {
            temp_vel = 0.3;
            temp_acc = 0.3;
            temp_tol = 0.01;
        }
        else if (planner_id == "LIN")
        {
            temp_vel = 0.2;
            temp_acc = 0.2;
            temp_tol = 0.01;
        }
        else
        {
            config["planner_id"] = "unknown";
        }
    }
    else if (pipeline_name == "ompl")
    {
        if (planner_id == "RRTConnect")
        {
            temp_vel = 0.2;
            temp_acc = 0.2;
            temp_tol = 0.01;
        }
        else if (planner_id == "PRM")
        {
            temp_vel = 0.3;
            temp_acc = 0.3;
            temp_tol = 0.01;
        }
        else
        {
            config["planner_id"] = "unknown";
        }
    }
    else
    {
        config["pipeline_name"] = "unknown";
    }

    if (max_vel_factor > 0.0) {
        temp_vel = max_vel_factor;
    }
    if (max_acc_factor > 0.0) {
        temp_acc = max_acc_factor;
    }
    if (tolerance > 0.0) {
        temp_tol = tolerance;
    }

    // Finally store them in config
    config["max_velocity_scaling_factor"] = temp_vel;
    config["max_acceleration_scaling_factor"] = temp_acc;
    config["tolerance"] = temp_tol;

    // If pipeline or planner is unknown, fail
    if (config["pipeline_name"].as<std::string>() == "unknown" ||
        config["planner_id"].as<std::string>() == "unknown")
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to save solver configuration: pipeline_name or planner_id is unknown");
        executed_ = false;
        return;
    }

    const std::string file_path = memory_dir + "current_solver_config.yaml";
    try {
        std::ofstream fout(file_path);
        fout << config;
        fout.close();
        RCLCPP_INFO(node_->get_logger(),
                    "Saved solver configuration to %s",
                    file_path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to save solver configuration: %s",
                     e.what());
        executed_ = false;
        return;
    }
    executed_ = true;
}

bool TaskBuilder::loadSolverConfig(const std::string& file_path)
{
    if (!fs::exists(file_path)) {
        RCLCPP_WARN(node_->get_logger(), "Solver configuration file %s does not exist.", file_path.c_str());
        executed_ = false;
        return false;
    }

    try {
        YAML::Node config = YAML::LoadFile(file_path);
        if (!config["pipeline_name"]) {
            RCLCPP_WARN(node_->get_logger(), "Missing 'pipeline_name' in config YAML.");
            executed_ = false;
            return false;
        }

        std::string pipeline_name = config["pipeline_name"].as<std::string>();
        RCLCPP_INFO(node_->get_logger(), "Loaded solver configuration: pipeline=%s", pipeline_name.c_str());

        if (!config["planner_id"]) {
            RCLCPP_WARN(node_->get_logger(), "Missing 'planner_id' in config YAML.");
            executed_ = false;
            return false;
        }

        std::string planner_id = config["planner_id"].as<std::string>();
        RCLCPP_INFO(node_->get_logger(), "Loaded solver configuration: planner_id=%s", planner_id.c_str());

        // We'll retrieve from YAML, or possibly reassign defaults if they are missing or NaN
        double vel_scale = 0.0;
        double acc_scale = 0.0;
        double tol_value = 0.0;

        // Now check if the YAML has explicitly set velocity, acceleration, or tolerance
        if (config["max_velocity_scaling_factor"]) {
            double from_yaml = config["max_velocity_scaling_factor"].as<double>();
            if (from_yaml > 0.0)
                vel_scale = from_yaml;
        }

        if (config["max_acceleration_scaling_factor"]) {
            double from_yaml = config["max_acceleration_scaling_factor"].as<double>();
            if (from_yaml > 0.0)
                acc_scale = from_yaml;
        }

        if (config["tolerance"]) {
            double from_yaml = config["tolerance"].as<double>();
            if (from_yaml > 0.0)
                tol_value = from_yaml;
        }

        // Choose the pipeline with final scale factors
        choosePipeline(pipeline_name, planner_id, vel_scale, acc_scale, tol_value);

        executed_ = true;
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load solver configuration: %s", e.what());
        executed_ = false;
        return false;
    }
}

void TaskBuilder::choosePipeline(const std::string& pipeline_name,
                                 const std::string& planner_id,
                                 double max_vel_factor,
                                 double max_acc_factor,
                                 double tolerance)
{
    // Clear existing solvers
    solvers_.clear();

    RCLCPP_INFO(node_->get_logger(),
                "[choose_pipeline] pipeline=%s, planner_id=%s",
                pipeline_name.c_str(), planner_id.c_str());

    if (pipeline_name == "pilz_industrial_motion_planner") {
        // Initialize Pilz PTP Planner
        auto pilz_ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
        pilz_ptp_planner->setPlannerId(planner_id.empty() ? "PTP" : planner_id);
        pilz_ptp_planner->setProperty("max_velocity_scaling_factor", max_vel_factor);
        pilz_ptp_planner->setProperty("max_acceleration_scaling_factor", max_acc_factor);
        solvers_["pilz_PTP"] = pilz_ptp_planner;

        // Initialize Pilz LIN Planner
        auto pilz_lin_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
        pilz_lin_planner->setPlannerId(planner_id.empty() ? "LIN" : planner_id);
        pilz_lin_planner->setProperty("max_velocity_scaling_factor", max_vel_factor);
        pilz_lin_planner->setProperty("max_acceleration_scaling_factor", max_acc_factor);
        solvers_["pilz_LIN"] = pilz_lin_planner;

        // Initialize Pilz CIRC Planner
        auto pilz_circ_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
        pilz_circ_planner->setPlannerId(planner_id.empty() ? "CIRC" : planner_id);
        pilz_circ_planner->setProperty("max_velocity_scaling_factor", max_vel_factor);
        pilz_circ_planner->setProperty("max_acceleration_scaling_factor", max_acc_factor);
        solvers_["pilz_CIRC"] = pilz_circ_planner;

        RCLCPP_INFO(node_->get_logger(), "Initialized Pilz solvers: PTP, LIN, CIRC");

        // Set a default active solver
        if (!solvers_.empty()) {
            current_solver_ = solvers_.begin()->second;
            RCLCPP_INFO(node_->get_logger(), "Set active solver to %s", solvers_.begin()->first.c_str());
        }
    }
    else if (pipeline_name == "ompl") {
        // Initialize OMPL Planner
        auto ompl_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
        ompl_planner->setPlannerId(planner_id.empty() ? "RRTConnectkConfigDefault" : planner_id);
        ompl_planner->setProperty("max_velocity_scaling_factor", max_vel_factor);
        ompl_planner->setProperty("max_acceleration_scaling_factor", max_acc_factor);
        solvers_["ompl_RRTConnect"] = ompl_planner;

        RCLCPP_INFO(node_->get_logger(), "Initialized OMPL solver: RRTConnect");

        // Set a default active solver
        current_solver_ = ompl_planner;
        RCLCPP_INFO(node_->get_logger(), "Set active solver to ompl_RRTConnect");
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Unknown pipeline name: %s", pipeline_name.c_str());
        executed_ = false;
        return;
    }

    solver_tolerance_ = tolerance;
    executed_ = true;
}

void TaskBuilder::printRobotParams() const
{
    auto robot_model = task_.getRobotModel();
    if (!robot_model) {
        RCLCPP_WARN(node_->get_logger(),
                    "No robot model is currently loaded. "
                    "Please call initTask() before querying robot parameters.");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "****************************************************");
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", robot_model->getModelFrame().c_str());

    RCLCPP_INFO(node_->get_logger(), "****************************************************");
    RCLCPP_INFO(node_->get_logger(), "Root link: %s", robot_model->getRootLinkName().c_str());

    const moveit::core::JointModelGroup* jmg =
        robot_model->getJointModelGroup(arm_group_name_);
    RCLCPP_INFO(node_->get_logger(), "****************************************************");
    RCLCPP_INFO(node_->get_logger(), "Active joints in group '%s':", arm_group_name_.c_str());
    if (jmg) {
        for (const auto& aj : jmg->getActiveJointModelNames())
            RCLCPP_INFO(node_->get_logger(), "  %s", aj.c_str());
    } else {
        RCLCPP_WARN(node_->get_logger(),
                    "JointModelGroup '%s' not found. Check MoveIt config.",
                    arm_group_name_.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "****************************************************");
    RCLCPP_INFO(node_->get_logger(), "All joint names in the robot:");
    for (const auto& joint_name : robot_model->getJointModelNames())
        RCLCPP_INFO(node_->get_logger(), "  %s", joint_name.c_str());

    RCLCPP_INFO(node_->get_logger(), "****************************************************");
    RCLCPP_INFO(node_->get_logger(), "All link names in the robot:");
    for (const auto& link_name : robot_model->getLinkModelNames())
        RCLCPP_INFO(node_->get_logger(), "  %s", link_name.c_str());

    RCLCPP_INFO(node_->get_logger(), "****************************************************");
    RCLCPP_INFO(node_->get_logger(), "Defined groups in SRDF:");
    for (const auto& group_name : robot_model->getJointModelGroupNames())
        RCLCPP_INFO(node_->get_logger(), "  %s", group_name.c_str());

    RCLCPP_INFO(node_->get_logger(), "****************************************************");
    RCLCPP_INFO(node_->get_logger(), "Robot markers: (no direct single call in MoveIt C++).");

    // ------------------------------------------------------------------
    // Wait for joint states if they haven't been received yet
    // ------------------------------------------------------------------
    RCLCPP_INFO(node_->get_logger(), "****************************************************");
    RCLCPP_INFO(node_->get_logger(), "Live Joint Positions (from /joint_states subscriber):");

    constexpr int max_retries = 5;
    constexpr int sleep_ms = 500;
    for (int attempt = 0; attempt < max_retries && current_joint_positions_.empty(); ++attempt) {
        rclcpp::spin_some(node_);
        if (current_joint_positions_.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No joint states received yet. Retrying...");
            rclcpp::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }
    if (current_joint_positions_.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to receive joint states after %d retries. "
                     "Check if the robot driver is publishing on /joint_states.",
                     max_retries);
        return;
    }

    for (const auto& kv : current_joint_positions_) {
        RCLCPP_INFO(node_->get_logger(), "  %s: %f", kv.first.c_str(), kv.second);
    }
    const_cast<TaskBuilder*>(this)->executed_ = true;
}

void TaskBuilder::clearScene()
{
    RCLCPP_INFO(node_->get_logger(), "[clear_scene] Called");

    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<std::string> object_ids = psi.getKnownObjectNames();

    if (object_ids.empty()) {
        RCLCPP_INFO(node_->get_logger(), "No collision objects found in the scene to remove.");
        executed_ = false;
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Removing %zu collision object(s) from the scene.", object_ids.size());
    psi.removeCollisionObjects(object_ids);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    std::vector<std::string> remaining_objects = psi.getKnownObjectNames();
    if (remaining_objects.empty()) {
        RCLCPP_INFO(node_->get_logger(), "All collision objects have been successfully removed from the scene.");
    } else {
        RCLCPP_WARN(node_->get_logger(), "Some collision objects could not be removed:");
        for (const auto& obj : remaining_objects) {
            RCLCPP_WARN(node_->get_logger(), " - %s", obj.c_str());
        }
    }
    executed_ = true;
}

// If you want to store each object in a more detailed structure:
static std::map<std::string, TaskBuilder::StoredObjectData> loadAllObjectData(const std::string& file_path)
{
    std::map<std::string, TaskBuilder::StoredObjectData> objects_map;

    if (!fs::exists(file_path))
        return objects_map;  // return empty if no file yet

    try {
        YAML::Node root = YAML::LoadFile(file_path);
        if (!root["objects"]) 
            return objects_map;

        YAML::Node objects_node = root["objects"];
        // Loop over each object name
        for (auto it = objects_node.begin(); it != objects_node.end(); ++it) {
            std::string obj_name = it->first.as<std::string>();
            YAML::Node data_node = it->second;

            TaskBuilder::StoredObjectData data;
            
            // Pose
            if (data_node["pose"]) {
                data.pose.position.x = data_node["pose"]["x"].as<double>();
                data.pose.position.y = data_node["pose"]["y"].as<double>();
                data.pose.position.z = data_node["pose"]["z"].as<double>();
                data.pose.orientation.x = data_node["pose"]["rx"].as<double>();
                data.pose.orientation.y = data_node["pose"]["ry"].as<double>();
                data.pose.orientation.z = data_node["pose"]["rz"].as<double>();
                data.pose.orientation.w = data_node["pose"]["rw"].as<double>();
            }

            // Shape
            if (data_node["shape"]) {
                data.shape = data_node["shape"].as<std::string>();
            }

            // Dimensions
            if (data_node["dimensions"] && data_node["dimensions"].IsSequence()) {
                for (auto dim : data_node["dimensions"]) {
                    data.dimensions.push_back(dim.as<double>());
                }
            }

            // Color
            if (data_node["color"]) {
                data.color = data_node["color"].as<std::string>();
            }

            // Alpha
            if (data_node["alpha"]) {
                data.alpha = data_node["alpha"].as<double>();
            } else {
                data.alpha = 1.0;  // default to opaque
            }

            objects_map[obj_name] = data;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[loadAllObjectData] Error reading file: " << e.what() << std::endl;
    }
    return objects_map;
}

static void saveAllObjectData(const std::string& file_path,
                              const std::map<std::string, TaskBuilder::StoredObjectData>& objects_map)
{
    YAML::Node root;
    YAML::Node objects_node;

    // If file already exists, try to parse existing data
    if (fs::exists(file_path)) {
        try {
            root = YAML::LoadFile(file_path);
            if (!root["objects"]) {
                root["objects"] = YAML::Node(YAML::NodeType::Map);
            }
            objects_node = root["objects"];
        }
        catch (const std::exception& e) {
            std::cerr << "[saveAllObjectData] Warning: Could not parse existing YAML. Overwriting. " 
                      << e.what() << std::endl;
            root = YAML::Node();
            objects_node = YAML::Node(YAML::NodeType::Map);
        }
    } else {
        objects_node = YAML::Node(YAML::NodeType::Map);
    }

    // Overwrite or insert data for each object
    for (const auto& [obj_name, data] : objects_map) {
        objects_node[obj_name]["pose"]["x"]  = data.pose.position.x;
        objects_node[obj_name]["pose"]["y"]  = data.pose.position.y;
        objects_node[obj_name]["pose"]["z"]  = data.pose.position.z;
        objects_node[obj_name]["pose"]["rx"] = data.pose.orientation.x;
        objects_node[obj_name]["pose"]["ry"] = data.pose.orientation.y;
        objects_node[obj_name]["pose"]["rz"] = data.pose.orientation.z;
        objects_node[obj_name]["pose"]["rw"] = data.pose.orientation.w;

        objects_node[obj_name]["shape"] = data.shape;

        // Save dimensions as a list: [da, db, dc...]
        YAML::Node dims_node(YAML::NodeType::Sequence);
        for (double d : data.dimensions) {
            dims_node.push_back(d);
        }
        objects_node[obj_name]["dimensions"] = dims_node;

        objects_node[obj_name]["color"] = data.color;
        objects_node[obj_name]["alpha"] = data.alpha;
    }

    root["objects"] = objects_node;

    // Write back to file
    try {
        std::ofstream fout(file_path);
        fout << root;
        fout.close();
    }
    catch (const std::exception& e) {
        std::cerr << "[saveAllObjectData] Error writing file: " << e.what() << std::endl;
    }
}

void TaskBuilder::removeObject(const std::string& object_name)
{
    RCLCPP_INFO(node_->get_logger(), "[remove_object] Removing %s", object_name.c_str());
    moveit::planning_interface::PlanningSceneInterface psi;

    // Check existence
    auto known_objects = psi.getKnownObjectNames();
    if (std::find(known_objects.begin(), known_objects.end(), object_name) == known_objects.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Object '%s' not found in the scene.", object_name.c_str());
        executed_ = false;
        return;
    }

    // Get the collision object from the scene
    auto object_map = psi.getObjects({object_name});
    if (object_map.find(object_name) == object_map.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Object '%s' not found in the scene map.", object_name.c_str());
        executed_ = false;
        return;
    }
    
    const moveit_msgs::msg::CollisionObject& obj_msg = object_map[object_name];

    // Log
    RCLCPP_INFO(node_->get_logger(), "Object ID: %s", obj_msg.id.c_str());
    RCLCPP_INFO(node_->get_logger(), "Frame ID: %s", obj_msg.header.frame_id.c_str());
    RCLCPP_INFO(node_->get_logger(), "Object operation: %d", obj_msg.operation);

    // Remove from the scene
    psi.removeCollisionObjects({object_name});
    RCLCPP_INFO(node_->get_logger(), "Object '%s' removed from scene.", object_name.c_str());

    // --- Gather data for saving to YAML ---
    StoredObjectData data;

    // 1) Pose
    if (!obj_msg.primitive_poses.empty()) {
        data.pose = obj_msg.pose;  // or obj_msg.primitive_poses[0], depending on how it was spawned
    }

    // 2) Shape and dimensions
    if (!obj_msg.primitives.empty()) {
        auto& prim = obj_msg.primitives[0];
        switch(prim.type) {
          case shape_msgs::msg::SolidPrimitive::BOX:
            data.shape = "box";
            break;
          case shape_msgs::msg::SolidPrimitive::SPHERE:
            data.shape = "sphere";
            break;
          case shape_msgs::msg::SolidPrimitive::CYLINDER:
            data.shape = "cylinder";
            break;
          case shape_msgs::msg::SolidPrimitive::CONE:
            data.shape = "cone";
            break;
          default:
            data.shape = "unknown";
        }
        for (double d : prim.dimensions) {
            data.dimensions.push_back(d);
        }
    }

    // 3) Default color & alpha
    data.color = "#FFFFFF";
    data.alpha = 1.0;

    // Load existing data, update or insert
    std::string package_share_directory;
    try {
        package_share_directory = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to get package share directory for 'fairino_mtc_demo': %s",
                     e.what());
        executed_ = false;
        return;
    }
    std::string memory_dir = package_share_directory + "/memory/";
    if (!fs::exists(memory_dir)) {
        try {
            fs::create_directories(memory_dir);
        } catch (const fs::filesystem_error& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create memory directory: %s", e.what());
            executed_ = false;
            return;
        }
    }
    std::string object_data_file = memory_dir + "object_data.yaml";

    // Load existing
    auto current_data = loadAllObjectData(object_data_file);
    // Insert/overwrite
    current_data[object_name] = data;
    // Save
    saveAllObjectData(object_data_file, current_data);

    RCLCPP_INFO(node_->get_logger(), "Saved object '%s' info (pose, shape, dims) to %s", 
                object_name.c_str(), object_data_file.c_str());
    executed_ = true;
}

void TaskBuilder::spawn_virtual_base(bool enable_virtual_base)
{
    std::string base_name =  "virtual_base";
    moveit::planning_interface::PlanningSceneInterface psi;
    // Check if there're any objects with the name and if yes then delete it
    if (!enable_virtual_base) {
        auto object_map = psi.getObjects({base_name});
        if (object_map.find(base_name) != object_map.end()) {
            psi.removeCollisionObjects({base_name});
            RCLCPP_INFO(node_->get_logger(), "[spawn_virtual_base] Virtual base removed");
            return;
        }
    }
    else {
        RCLCPP_INFO(node_->get_logger(), "[spawn_virtual_base] Spawning virtual base obstacle");
        // Define the collision object
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = base_name;
        collision_object.header.frame_id = "world";
    
        // Define a box primitive with dimensions 10 x 10 x 0.01
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 10.0;
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 10.0;
        primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.01;
    
        // Place the box so that its top is at z = 0.
        // (For a 0.01-m thick box, its center should be at z = -0.005.)
        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -0.01;
        box_pose.orientation.x = 0.0;
        box_pose.orientation.y = 0.0;
        box_pose.orientation.z = 0.0;
        box_pose.orientation.w = 1.0;
    
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
    
        // Apply the collision object to the planning scene
        psi.applyCollisionObject(collision_object);
    
        RCLCPP_INFO(node_->get_logger(), "[spawn_virtual_base] Virtual base spawned.");
    }

}

void TaskBuilder::spawnObject(const std::string& object_name,
                              const std::string& object_shape,
                              double x, double y, double z,
                              double rx, double ry, double rz, double rw,
                              double da, double db, double dc)
{
    RCLCPP_INFO(node_->get_logger(), "[spawn_object] Attempt to spawn '%s'", object_name.c_str());
    moveit::planning_interface::PlanningSceneInterface psi;

    // Get the collision object from the scene
    auto object_map = psi.getObjects({object_name});
    if (object_map.find(object_name) != object_map.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Object '%s' already in the map. Aborted!", object_name.c_str());
        executed_ = false;
        return;
    }

    bool shape_provided = !object_shape.empty() && object_shape != "use_memory";
    bool coordinates_provided = !(std::isnan(x) || std::isnan(y) || std::isnan(z) ||
                                  std::isnan(rx) || std::isnan(ry) || std::isnan(rz) || std::isnan(rw));

    // Build new struct with user input
    StoredObjectData data;
    mem_data.pose.position.x = x;
    mem_data.pose.position.y = y;
    mem_data.pose.position.z = z;
    mem_data.pose.orientation.x = rx;
    mem_data.pose.orientation.y = ry;
    mem_data.pose.orientation.z = rz;
    mem_data.pose.orientation.w = rw;
    mem_data.shape = object_shape;

    // dimensions
    if (object_shape == "cylinder" || object_shape == "cone") {
        mem_data.dimensions = {da, db};
    } else if (object_shape == "box") {
        mem_data.dimensions = {da, db, dc};
    } else if (object_shape == "sphere") {
        mem_data.dimensions = {da};
    }
    mem_data.color = "#FFFFFF";
    mem_data.alpha = 1.0;
    data = mem_data;
    
    // If shape or coords not provided, load from memory
    if (!shape_provided || !coordinates_provided) {
        RCLCPP_INFO(node_->get_logger(), "[spawn_object] shape/coords not fully provided; checking object_data.yaml.");

        std::string pkg_share;
        try {
            pkg_share = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
        } catch(...) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get share dir for fairino_mtc_demo. Aborting spawn.");
            executed_ = false;
            return;
        }
        std::string memory_dir = pkg_share + "/memory/";
        std::string data_file  = memory_dir + "object_data.yaml";

        auto existing_data = loadAllObjectData(data_file);
        if (existing_data.find(object_name) == existing_data.end()) {
            RCLCPP_ERROR(node_->get_logger(),
                "No extended data for '%s' found in memory file %s. Cannot spawn.",
                object_name.c_str(), data_file.c_str());
            executed_ = false;
            return;
        }
        // Overwrite with saved data
        mem_data = existing_data[object_name];
        data.pose = mem_data.pose;
        if (!shape_provided)
            data.shape = mem_data.shape;
        if (!mem_data.dimensions.empty())
            data.dimensions = mem_data.dimensions;
        // color, alpha, etc.
        data.color = mem_data.color;
        data.alpha = mem_data.alpha;
    }

    if (data.shape.empty() || data.shape == "unknown") {
        RCLCPP_ERROR(node_->get_logger(), "Shape not recognized. Abort spawn.");
        executed_ = false;
        return;
    }

    // Create collision object
    moveit_msgs::msg::CollisionObject object;
    object.id = object_name;
    object.header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive prim;
    if (data.shape == "cylinder") {
        prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        prim.dimensions.resize(2);
        // By convention: [height, radius]
        prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = data.dimensions[0];
        prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = data.dimensions[1];
    } else if (data.shape == "box") {
        prim.type = shape_msgs::msg::SolidPrimitive::BOX;
        prim.dimensions.resize(3);
        for (size_t i = 0; i < data.dimensions.size() && i < 3; ++i) {
            prim.dimensions[i] = data.dimensions[i];
        }
    } else if (data.shape == "sphere") {
        prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        prim.dimensions.resize(1);
        prim.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = data.dimensions[0];
    } else if (data.shape == "cone") {
        prim.type = shape_msgs::msg::SolidPrimitive::CONE;
        prim.dimensions.resize(2);
        // By convention: [height, radius]
        prim.dimensions[shape_msgs::msg::SolidPrimitive::CONE_HEIGHT] = data.dimensions[0];
        prim.dimensions[shape_msgs::msg::SolidPrimitive::CONE_RADIUS] = data.dimensions[1];
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Unsupported shape: %s", data.shape.c_str());
        executed_ = false;
        return;
    }
    object.primitives.push_back(prim);
    object.primitive_poses.push_back(data.pose);

    // Apply
    psi.applyCollisionObject(object);
    RCLCPP_INFO(node_->get_logger(),
                "Spawned object '%s' [shape=%s] at (%.2f,%.2f,%.2f).",
                object_name.c_str(), data.shape.c_str(),
                data.pose.position.x, data.pose.position.y, data.pose.position.z);

    // Save to object_data.yaml
    std::string pkg_share;
    try {
        pkg_share = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
    } catch(...) { 
        executed_ = false;
        return; 
    }
    std::string data_file = pkg_share + "/memory/object_data.yaml";
    auto all_data = loadAllObjectData(data_file);
    all_data[object_name] = data;
    saveAllObjectData(data_file, all_data);
    executed_ = true;
}

void TaskBuilder::jointsMove(const std::vector<double>& joint_values)
{
    if (joint_values.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Waiting for joint states...");
        constexpr int max_retries = 5;
        constexpr int sleep_ms = 500;
        for (int attempt = 0; attempt < max_retries; ++attempt) {
            rclcpp::spin_some(node_);
            if (!current_joint_positions_.empty()) {
                RCLCPP_INFO(node_->get_logger(), "Live Joint Positions:");
                for (const auto& joint_name : joint_names_) {
                    RCLCPP_INFO(node_->get_logger(), "  Joint %s: %f",
                                joint_name.c_str(), current_joint_positions_[joint_name]);
                }
                executed_ = true;
                return;
            }
            RCLCPP_WARN(node_->get_logger(), "No joint positions received yet. Retrying...");
            rclcpp::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        RCLCPP_ERROR(node_->get_logger(), "Failed to receive joint states after %d retries. Aborting.", max_retries);
        executed_ = false;
        return;
    }

    rclcpp::spin_some(node_);
    if (joint_values.size() != joint_names_.size()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "jointsMove() called with %zu values, but expects %zu joints",
                     joint_values.size(), joint_names_.size());
        executed_ = false;
        return;
    }

    std::map<std::string, double> joints_map;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // joints_map[joint_names_[i]] = joint_values[i]; // This is more versatile bit requires some sorting algorithm
        joints_map["j" + std::to_string(i+1)] = joint_values[i];
    }

    if (!current_solver_) {
        RCLCPP_ERROR(node_->get_logger(), "No active solver selected. Use choosePipeline() first.");
        executed_ = false;
        return;
    }

    auto stage = std::make_unique<mtc::stages::MoveTo>("move to joints", current_solver_);
    stage->setGroup(arm_group_name_);
    stage->setGoal(joints_map);
    stage->setTimeout(10.0);
    if (solver_tolerance_) {
        stage->properties().set("goal_tolerance", solver_tolerance_);
    }
    task_.add(std::move(stage));
    executed_ = true;
}

void TaskBuilder::absoluteMove(const std::string& frame_id, 
                               const std::string& tip_frame,
                               const std::string& target_frame,
                               double x, double y, double z,
                               double rx, double ry, double rz, double rw)
{
    geometry_msgs::msg::PoseStamped goal_pose_stamped;
    std::string task_name = "move_to_absolute_pose";
    
    // If tip_frame and target_frame are provided, check if the target object exists in the environment.
    if (!tip_frame.empty() && !target_frame.empty()) 
    {
        moveit::planning_interface::PlanningSceneInterface psi;
        auto known_objects = psi.getKnownObjectNames();
        if (std::find(known_objects.begin(), known_objects.end(), target_frame) == known_objects.end()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "[absolute_move] Object '%s' is not present in the environment. Aborting.",
                         target_frame.c_str());
            executed_ = false;
            return;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "[absolute_move] Target object '%s' found in environment. Attempting to load pose from memory or TF.",
                    target_frame.c_str());
        task_name = "move_to_frames";

        bool loaded_from_memory = false;
        try {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
            std::string memory_dir = pkg_share + "/memory/";
            std::string data_file  = memory_dir + "object_data.yaml";

            auto all_objects = loadAllObjectData(data_file);
            auto it = all_objects.find(target_frame);
            if (it != all_objects.end())
            {
                goal_pose_stamped.header.frame_id = frame_id;
                goal_pose_stamped.header.stamp = node_->now();
                goal_pose_stamped.pose = it->second.pose;
                loaded_from_memory = true;
                RCLCPP_INFO(node_->get_logger(),
                            "[absolute_move] Found '%s' in object_data.yaml. Using its pose as goal.",
                            target_frame.c_str());
            }
        }
        catch(const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(),
                         "[absolute_move] Exception checking object_data.yaml: %s", e.what());
        }

        if (!loaded_from_memory)
        {
            // Try TF lookup if the object data wasn't loaded from memory.
            RCLCPP_INFO(node_->get_logger(),
                        "[absolute_move] Attempting TF lookup from '%s' -> '%s'.",
                        frame_id.c_str(), target_frame.c_str());
            try {
                static tf2_ros::Buffer tf_buffer(node_->get_clock());
                static tf2_ros::TransformListener tf_listener(tf_buffer);

                geometry_msgs::msg::TransformStamped tf_transform = 
                    tf_buffer.lookupTransform(frame_id, target_frame,
                                              rclcpp::Time(0),
                                              rclcpp::Duration(1,0)); // 1s timeout

                goal_pose_stamped.header.frame_id = frame_id;
                goal_pose_stamped.header.stamp = node_->now();
                goal_pose_stamped.pose.position.x = tf_transform.transform.translation.x;
                goal_pose_stamped.pose.position.y = tf_transform.transform.translation.y;
                goal_pose_stamped.pose.position.z = tf_transform.transform.translation.z;
                goal_pose_stamped.pose.orientation = tf_transform.transform.rotation;

                RCLCPP_INFO(node_->get_logger(), "[absolute_move] TF lookup success. Using transform as goal.");
            }
            catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(node_->get_logger(),
                             "[absolute_move] TF lookup failed: %s", ex.what());
                RCLCPP_ERROR(node_->get_logger(),
                             "[absolute_move] Could NOT resolve pose for target_frame='%s'. Aborting.",
                             target_frame.c_str());
                executed_ = false;
                return;
            }
        }
    }
    else
    {
        // Use direct coordinates.
        if (std::isnan(x) || std::isnan(y) || std::isnan(z) ||
            std::isnan(rx) || std::isnan(ry) || std::isnan(rz) || std::isnan(rw))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[absolute_move] Invalid args: specify (tip_frame,target_frame) or (x,y,z,rx,ry,rz,rw).");
            executed_ = false;
            return;
        }
        
        RCLCPP_INFO(node_->get_logger(),
                    "[absolute_move] Using direct pose: (%.2f,%.2f,%.2f) / (%.2f,%.2f,%.2f,%.2f) in '%s'.",
                    x, y, z, rx, ry, rz, rw, frame_id.c_str());

        goal_pose_stamped.header.frame_id = frame_id;
        goal_pose_stamped.header.stamp = node_->now();
        goal_pose_stamped.pose.position.x = x;
        goal_pose_stamped.pose.position.y = y;
        goal_pose_stamped.pose.position.z = z;
        goal_pose_stamped.pose.orientation.x = rx;
        goal_pose_stamped.pose.orientation.y = ry;
        goal_pose_stamped.pose.orientation.z = rz;
        goal_pose_stamped.pose.orientation.w = rw;
    }

    if (!current_solver_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[absolute_move] No active solver set. Use choosePipeline() first.");
        executed_ = false;
        return;
    }

    auto stage = std::make_unique<mtc::stages::MoveTo>(task_name, current_solver_);
    stage->setGroup(arm_group_name_);
    stage->setTimeout(5.0);
    if (!tip_frame.empty())
        stage->setIKFrame(tip_frame);
    if (solver_tolerance_) {
        stage->properties().set("goal_tolerance", solver_tolerance_);
    }
    stage->setGoal(goal_pose_stamped);

    task_.add(std::move(stage));
    executed_ = true;
}

void TaskBuilder::displacementMove(const std::string& world_frame, 
                                   const std::string& tip_frame,
                                   const std::vector<double>& translation_vector,
                                   const std::vector<double>& rotation_vector)
{
    if (translation_vector.size() != 3 || rotation_vector.size() != 3) {
        RCLCPP_ERROR(node_->get_logger(),
                     "displacementMove() requires exactly 3 elements for translation and 3 for rotation");
        executed_ = false;
        return;
    }

    bool is_translation_zero = (translation_vector[0] == 0.0 && 
                                translation_vector[1] == 0.0 && 
                                translation_vector[2] == 0.0);
    bool is_rotation_zero = (rotation_vector[0] == 0.0 && 
                             rotation_vector[1] == 0.0 && 
                             rotation_vector[2] == 0.0);

    if (is_translation_zero && is_rotation_zero) {
        RCLCPP_WARN(node_->get_logger(), 
                    "Both translation and rotation vectors are zero. Nothing to do.");
        executed_ = false;
        return;
    }

    // Translation
    if (!is_translation_zero) {
        auto stage_translate = std::make_unique<mtc::stages::MoveRelative>("translate", cartesian_planner_);
        stage_translate->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage_translate->setMinMaxDistance(0.1, 0.2);
        stage_translate->setIKFrame(tip_frame);
        stage_translate->properties().set("marker_ns", "translate");
        if (solver_tolerance_) {
            stage_translate->properties().set("goal_tolerance", solver_tolerance_);
        }
        geometry_msgs::msg::Vector3Stamped translation;
        translation.header.frame_id = world_frame;
        translation.vector.x = translation_vector[0];
        translation.vector.y = translation_vector[1];
        translation.vector.z = translation_vector[2];
        stage_translate->setDirection(translation);

        task_.add(std::move(stage_translate));
    }

    // Rotation
    if (!is_rotation_zero) {
        auto stage_rotate = std::make_unique<mtc::stages::MoveRelative>("rotate", cartesian_planner_);
        stage_rotate->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage_rotate->setMinMaxDistance(0.01, 0.2);
        stage_rotate->setIKFrame(tip_frame);
        stage_rotate->properties().set("marker_ns", "rotate");
        if (solver_tolerance_) {
            stage_rotate->properties().set("goal_tolerance", solver_tolerance_);
        }
        geometry_msgs::msg::TwistStamped rotation;
        rotation.header.frame_id = world_frame;
        rotation.twist.angular.x = rotation_vector[0];
        rotation.twist.angular.y = rotation_vector[1];
        rotation.twist.angular.z = rotation_vector[2];
        stage_rotate->setDirection(rotation);

        task_.add(std::move(stage_rotate));
    }
    executed_ = true;
}

void TaskBuilder::trajectoryMove(const std::string& csv_file,
                                 double velocity_scale,
                                 double accel_scale,
                                 double pose_tolerance)
{
    RCLCPP_INFO(node_->get_logger(),
                "[trajectoryMove] CSV: %s, vel=%.2f, accel=%.2f, tol=%.2f",
                csv_file.c_str(), velocity_scale, accel_scale, pose_tolerance);

    // Parse CSV
    std::vector<geometry_msgs::msg::PoseStamped> waypoints = parseCsv(csv_file);
    if (waypoints.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "No valid waypoints from CSV. Aborting trajectoryMove.");
        executed_ = false;
        return;
    }

    // Instead of direct computeCartesianPath + execution, just add these waypoints as MTC stages:
    planWaypoints(waypoints, velocity_scale, accel_scale, pose_tolerance);
    // (Remember: "planWaypoints" below now only ADDS the stages; it does not run them.)
    executed_ = true;
}

void TaskBuilder::feedbackPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    RCLCPP_INFO(node_->get_logger(),
                "[feedbackPoseCallback] Received new target Pose: (%.2f, %.2f, %.2f)",
                msg->pose.position.x, 
                msg->pose.position.y, 
                msg->pose.position.z);

    std::vector<geometry_msgs::msg::PoseStamped> single_target;
    single_target.push_back(*msg);

    // Just add the stage (no direct execution)
    planWaypoints(single_target, 0.5, 0.5, 0.01);
    executed_ = true;
}

void TaskBuilder::feedbackMove(const std::string& pose_topic)
{
    RCLCPP_INFO(node_->get_logger(), "[feedbackMove] Subscribing to: %s", pose_topic.c_str());

    feedback_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, 
        10, 
        std::bind(&TaskBuilder::feedbackPoseCallback, this, std::placeholders::_1));

    // Example: wait 10s for any incoming poses
    rclcpp::Time start_time = node_->now();
    while ((node_->now() - start_time).seconds() < 10.0) {
        rclcpp::spin_some(node_);
    }

    RCLCPP_INFO(node_->get_logger(),
                "feedback_move: done waiting. Unsubscribing from %s.",
                pose_topic.c_str());
    feedback_sub_.reset();
    executed_ = true;
}

void TaskBuilder::attachObject(const std::string& object_name, const std::string& link_name)
{
    // Use the PlanningSceneInterface to check for attached objects
    moveit::planning_interface::PlanningSceneInterface psi;
    auto attached_objects = psi.getAttachedObjects({object_name});
    if (!attached_objects.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Already attached body '%s'.", object_name.c_str());
        executed_ = false;
        return;
    }

    // Get the collision object from the scene
    auto object_map = psi.getObjects({object_name});
    if (object_map.find(object_name) == object_map.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Object '%s' not found in the scene map.", object_name.c_str());
        executed_ = false;
        return;
    }

    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject(object_name, link_name);
    task_.add(std::move(stage));
    executed_ = true;
}

void TaskBuilder::detachObject(const std::string& object_name, const std::string& link_name)
{
    // Use the PlanningSceneInterface to check for attached objects
    moveit::planning_interface::PlanningSceneInterface psi;
    auto attached_objects = psi.getAttachedObjects({object_name});
    if (attached_objects.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Attached body '%s' not found. Aborting operation.", object_name.c_str());
        executed_ = false;
        return;
    }

    // Retrieve the attached object's data
    auto it = attached_objects.find(object_name);
    if (it == attached_objects.end()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Attached body '%s' not found in attached objects map. Aborting operation.", object_name.c_str());
        executed_ = false;
        return;
    }
    const auto &attached_obj = it->second;
    const moveit_msgs::msg::CollisionObject& obj_msg = attached_obj.object;

    // --- Gather data for saving to YAML (similar to removeObject) ---
    StoredObjectData data;
    if (!obj_msg.primitive_poses.empty()) {
        data.pose = obj_msg.pose; // Alternatively, use obj_msg.primitive_poses[0]
    }
    if (!obj_msg.primitives.empty()) {
        const auto &prim = obj_msg.primitives[0];
        switch(prim.type) {
          case shape_msgs::msg::SolidPrimitive::BOX:
            data.shape = "box";
            break;
          case shape_msgs::msg::SolidPrimitive::SPHERE:
            data.shape = "sphere";
            break;
          case shape_msgs::msg::SolidPrimitive::CYLINDER:
            data.shape = "cylinder";
            break;
          case shape_msgs::msg::SolidPrimitive::CONE:
            data.shape = "cone";
            break;
          default:
            data.shape = "unknown";
        }
        for (double d : prim.dimensions) {
            data.dimensions.push_back(d);
        }
    }
    data.color = "#FFFFFF";
    data.alpha = 1.0;

    // Save the object's data to YAML
    std::string package_share_directory;
    try {
        package_share_directory = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to get package share directory for 'fairino_mtc_demo': %s", e.what());
        executed_ = false;
        return;
    }
    std::string memory_dir = package_share_directory + "/memory/";
    if (!fs::exists(memory_dir)) {
        try {
            fs::create_directories(memory_dir);
        } catch (const fs::filesystem_error& e) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Failed to create memory directory: %s", e.what());
            executed_ = false;
            return;
        }
    }
    std::string object_data_file = memory_dir + "object_data.yaml";
    auto current_data = loadAllObjectData(object_data_file);
    current_data[object_name] = data;
    saveAllObjectData(object_data_file, current_data);
    RCLCPP_INFO(node_->get_logger(), "Saved detached object '%s' info to %s", 
                object_name.c_str(), object_data_file.c_str());

    // Now build and add the detach stage
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject(object_name, link_name);
    task_.add(std::move(stage));
    executed_ = true;
}

void TaskBuilder::toolControl(const ToolControlConfig& config)
{
    RCLCPP_INFO(node_->get_logger(), "[tool_control] Called");
    RCLCPP_INFO(node_->get_logger(), "[tool_control] Mode: %s", config.mode.c_str());
    RCLCPP_INFO(node_->get_logger(), "[tool_control] feedback_enabled: %f", config.feedback_enabled);
    RCLCPP_ERROR(node_->get_logger(), "[tool_control] have no action implemented yet.");
    executed_ = false;
}

void TaskBuilder::collaborativeMove(const std::string& torque_topic, const std::string& record_filename)
{
    RCLCPP_INFO(node_->get_logger(),
                "[collaborativeMove] torque_topic=%s, record=%s",
                torque_topic.c_str(), record_filename.c_str());

    torque_sub_ = node_->create_subscription<std_msgs::msg::String>(
        torque_topic, 10,
        std::bind(&TaskBuilder::torqueFeedbackCallback, this, std::placeholders::_1));

    rclcpp::Time start_time = node_->now();
    while ((node_->now() - start_time).seconds() < 5.0) {
        rclcpp::spin_some(node_);
        geometry_msgs::msg::Pose dummy_pose;
        dummy_pose.position.x = 0.0;
        dummy_pose.position.y = 0.0;
        dummy_pose.position.z = 0.0;
        dummy_pose.orientation = initial_orientation_;
        recordPose(dummy_pose, record_filename);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(node_->get_logger(),
                "collaborative_move: finished. Motion was recorded to %s.",
                record_filename.c_str());
    executed_ = true;
}

void TaskBuilder::torqueFeedbackCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node_->get_logger(),
                "[torqueFeedbackCallback] Received torque feedback: %s",
                msg->data.c_str());
    executed_ = true;
}

void TaskBuilder::recordPose(const geometry_msgs::msg::Pose& pose, const std::string& filename)
{
    std::ofstream outfile(filename, std::ios::app);
    if (!outfile.is_open()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Could not open file %s for recording pose", filename.c_str());
        executed_ = false;
        return;
    }
    outfile << pose.position.x << "," << pose.position.y << "," << pose.position.z << ","
            << pose.orientation.x << "," << pose.orientation.y << "," << pose.orientation.z << "," << pose.orientation.w
            << "\n";
    outfile.close();
    RCLCPP_INFO(node_->get_logger(), "Recorded pose to %s", filename.c_str());
    executed_ = true;
}

std::vector<geometry_msgs::msg::Pose> TaskBuilder::gcodeLoad(const std::string& gcode_file,
                                                             const std::string& mode)
{
    std::vector<GCodeCommand> all_commands;
    std::ifstream file(gcode_file);
    if (!file.is_open())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open G-code file: %s", gcode_file.c_str());
        executed_ = false;
        return {};
    }
    std::string raw_line;
    while (std::getline(file, raw_line))
    {
        if (raw_line.empty())
            continue;
        GCodeCommand cmd = parseGCodeLine(raw_line);
        if (!cmd.code.empty() || !cmd.params.empty())
            all_commands.push_back(cmd);
    }
    file.close();
    
    if (mode == "printing") {
        RCLCPP_INFO(node_->get_logger(), "Mode: %s", mode.c_str());
    }
    else if  (mode == "milling") {
        RCLCPP_INFO(node_->get_logger(), "Mode: %s", mode.c_str());
    }
    else if  (mode == "welding") {
        RCLCPP_INFO(node_->get_logger(), "Mode: %s", mode.c_str());
    }

    double x = 0.0, y = 0.0, z = 0.0;
    std::vector<geometry_msgs::msg::Pose> path;

    RCLCPP_ERROR(node_->get_logger(), "THE ORIENTATION IS THE DEFAULT!");
    for (auto& c : all_commands)
    {
        if (c.code == "G0" || c.code == "G1")
        {
            if (c.params.find('X') != c.params.end())
                x = c.params.at('X');
            if (c.params.find('Y') != c.params.end())
                y = c.params.at('Y');
            if (c.params.find('Z') != c.params.end())
                z = c.params.at('Z');

            geometry_msgs::msg::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;
            path.push_back(pose);
        }
    }
    executed_ = true;
    return path;
}

std::vector<geometry_msgs::msg::Pose> TaskBuilder::stepLoad(const std::string& step_file)
{
    std::vector<geometry_msgs::msg::Pose> result;
    std::ifstream file(step_file);
    if (!file.is_open())
    {
        RCLCPP_ERROR(node_->get_logger(), "stepLoad: could not open file: %s", step_file.c_str());
        executed_ = false;
        return result;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line))
        lines.push_back(line);
    file.close();

    // A simple example of searching for B_SPLINE_CURVE_WITH_KNOTS
    std::string spline_ref;
    for (const auto &l : lines)
    {
        if (l.find("B_SPLINE_CURVE_WITH_KNOTS") != std::string::npos)
        {
            spline_ref = l;
            break;
        }
    }
    if (spline_ref.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "No B_SPLINE_CURVE_WITH_KNOTS found in %s", step_file.c_str());
        executed_ = false;
        return result;
    }

    std::vector<std::string> cartesian_refs;
    {
        auto startPos = spline_ref.find("(#");
        if (startPos == std::string::npos)
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not parse control points list.");
            executed_ = false;
            return result;
        }
        auto endPos = spline_ref.find(")", startPos);
        if (endPos == std::string::npos)
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not parse control points list (no closing parenthesis).");
            executed_ = false;
            return result;
        }
        std::string refs = spline_ref.substr(startPos, endPos - startPos);
        refs.erase(std::remove(refs.begin(), refs.end(), '('), refs.end());
        refs.erase(std::remove(refs.begin(), refs.end(), ')'), refs.end());
        std::stringstream ss(refs);
        std::string token;
        while (std::getline(ss, token, ','))
        {
            if (!token.empty() && token.find("#") != std::string::npos)
            {
                while (!token.empty() && std::isspace(token.front())) token.erase(token.begin());
                while (!token.empty() && std::isspace(token.back())) token.pop_back();
                cartesian_refs.push_back(token);
            }
        }
    }
    if (cartesian_refs.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "No cartesian point references found.");
        executed_ = false;
        return result;
    }
    struct Point3 { double x, y, z; };
    std::vector<Point3> control_points;
    for (const auto &ref : cartesian_refs)
    {
        for (const auto &l : lines)
        {
            if (l.rfind(ref + "=", 0) == 0 && l.find("CARTESIAN_POINT") != std::string::npos)
            {
                auto cpointPos = l.find("CARTESIAN_POINT(");
                if (cpointPos == std::string::npos)
                    continue;
                cpointPos += std::strlen("CARTESIAN_POINT(");
                auto secondParen = l.find("(", cpointPos);
                if (secondParen == std::string::npos)
                    continue;
                auto endParen = l.find(")", secondParen + 1);
                if (endParen == std::string::npos)
                    continue;
                std::string coords = l.substr(secondParen + 1, endParen - (secondParen + 1));
                std::stringstream ssc(coords);
                std::string val;
                std::vector<double> vals;
                while (std::getline(ssc, val, ','))
                {
                    while (!val.empty() && std::isspace(val.front())) val.erase(val.begin());
                    while (!val.empty() && std::isspace(val.back())) val.pop_back();
                    try {
                        vals.push_back(std::stod(val));
                    } catch (...) { }
                }
                if (vals.size() == 3)
                    control_points.push_back({vals[0], vals[1], vals[2]});
                break;
            }
        }
    }
    if (control_points.size() < 2)
    {
        RCLCPP_ERROR(node_->get_logger(), "Not enough control points to form a curve.");
        executed_ = false;
        return result;
    }
    // Simple linear interpolation among control points
    int num_samples = 20;
    RCLCPP_ERROR(node_->get_logger(), "THE ORIENTATION IS THE DEFAULT!");
    for (int i = 0; i < num_samples; ++i)
    {
        double t = static_cast<double>(i) / (num_samples - 1);
        double float_index = t * (control_points.size() - 1);
        int idx0 = static_cast<int>(std::floor(float_index));
        int idx1 = std::min(idx0 + 1, (int)control_points.size() - 1);
        double ratio = float_index - idx0;

        double x = control_points[idx0].x + ratio * (control_points[idx1].x - control_points[idx0].x);
        double y = control_points[idx0].y + ratio * (control_points[idx1].y - control_points[idx0].y);
        double z = control_points[idx0].z + ratio * (control_points[idx1].z - control_points[idx0].z);

        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        result.push_back(pose);
    }
    executed_ = true;
    return result;
}

// Force usage of Pilz LIN for a straight line. Use the orientation from initTask.
void TaskBuilder::scanLine(std::string world_frame, std::string tip_frame,
                           const geometry_msgs::msg::Point& start,
                           const geometry_msgs::msg::Point& end)
{
    RCLCPP_INFO(node_->get_logger(),
                "[scanLine] from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
                start.x, start.y, start.z,
                end.x,   end.y,   end.z);

    // ---------------------------------------------------------
    // Capture the robot's current tip-frame orientation via TF
    // from the manipulator's base (e.g., "base_link") to the end effector.
    // ---------------------------------------------------------
    static tf2_ros::Buffer tf_buffer(node_->get_clock());
    static tf2_ros::TransformListener tf_listener(tf_buffer);

    try {
        // Look up the transform from the base frame to the tip frame.
        // Adjust "base_link" if your base frame has a different name.
        geometry_msgs::msg::TransformStamped tf_transform =
            tf_buffer.lookupTransform("world", "tip_link", rclcpp::Time(0), rclcpp::Duration(1, 0));

        initial_orientation_.x = tf_transform.transform.rotation.x;
        initial_orientation_.y = tf_transform.transform.rotation.y;
        initial_orientation_.z = tf_transform.transform.rotation.z;
        initial_orientation_.w = tf_transform.transform.rotation.w;
        RCLCPP_INFO(node_->get_logger(),
                    "[initTask] Obtained initial orientation from TF: (%.3f, %.3f, %.3f, %.3f)",
                    initial_orientation_.x,
                    initial_orientation_.y,
                    initial_orientation_.z,
                    initial_orientation_.w);
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "[initTask] TF lookup failed: %s", ex.what());
        // Fallback to identity orientation if the lookup fails.
        executed_ = false;
    }

    //
    // 1) Create local planners: OMPL & Pilz LIN
    //    (Here we choose some default velocity/acceleration scaling;
    //     adjust as desired or make them parameters.)
    //
    double max_vel_factor = 0.1;
    double max_acc_factor = 0.1;

    // OMPL planner (for first point)
    auto ompl_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    ompl_planner->setPlannerId("RRTConnect");  // or "RRTstar", etc.
    ompl_planner->setProperty("max_velocity_scaling_factor", max_vel_factor);
    ompl_planner->setProperty("max_acceleration_scaling_factor", max_acc_factor);

    // Pilz LIN planner (for straight line from A to B)
    auto pilz_lin_planner =
        std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
    pilz_lin_planner->setPlannerId("LIN");
    pilz_lin_planner->setProperty("max_velocity_scaling_factor", max_vel_factor);
    pilz_lin_planner->setProperty("max_acceleration_scaling_factor", max_acc_factor);

    //
    // 2) Build two PoseStamped goals (A and B) with the stored orientation
    //
    geometry_msgs::msg::PoseStamped stampedA, stampedB;
    stampedA.header.frame_id = world_frame;
    stampedA.header.stamp    = node_->now();
    stampedA.pose.position   = start;
    stampedA.pose.orientation.x = initial_orientation_.x;
    stampedA.pose.orientation.y = initial_orientation_.y;
    stampedA.pose.orientation.z = initial_orientation_.z;
    stampedA.pose.orientation.w = initial_orientation_.w;

    std::cout << initial_orientation_.x << std::endl;
    std::cout << initial_orientation_.y << std::endl;
    std::cout << initial_orientation_.z << std::endl;
    std::cout << initial_orientation_.w << std::endl;

    stampedB.header.frame_id = world_frame;
    stampedB.header.stamp    = node_->now();
    stampedB.pose.position   = end;
    stampedB.pose.orientation.x = initial_orientation_.x;
    stampedB.pose.orientation.y = initial_orientation_.y;
    stampedB.pose.orientation.z = initial_orientation_.z;
    stampedB.pose.orientation.w = initial_orientation_.w;

    // Move from current state to the first point (OMPL)
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("move to start (OMPL)", ompl_planner);
        stage->setGroup(arm_group_name_);
        stage->setIKFrame(tip_frame);
        stage->setGoal(stampedA);
        stage->setTimeout(5.0);
        task_.add(std::move(stage));
    }

    // Move from first point to second point (Pilz LIN)
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("move to end (LIN)", pilz_lin_planner);
        stage->setGroup(arm_group_name_);
        stage->setIKFrame(tip_frame);
        stage->setGoal(stampedB);
        stage->setTimeout(5.0);
        task_.add(std::move(stage));
    }

    RCLCPP_INFO(node_->get_logger(),
                "[scanLine] Added 1) OMPL move to start, then 2) LIN move to end. "
                "Call planTask() + executeTask() to run.");
    executed_ = true;
}



// generate a random reachable {x,y,z} (once only) and orient the +Z axis toward the given (x, y, z).
void TaskBuilder::calibrateCamera(std::string tip_frame, double x, double y, double z)
{
    static bool random_already_generated = false;
    static geometry_msgs::msg::Point randomPos;

    if (!random_already_generated) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> distX(0.3, 0.6);
        std::uniform_real_distribution<double> distY(-0.2, 0.2);
        std::uniform_real_distribution<double> distZ(0.2, 0.5);

        randomPos.x = distX(gen);
        randomPos.y = distY(gen);
        randomPos.z = distZ(gen);

        RCLCPP_INFO(node_->get_logger(),
                    "[calibrateCamera] Generating random pos: (%.3f, %.3f, %.3f)",
                    randomPos.x, randomPos.y, randomPos.z);

        random_already_generated = true;
    } else {
        RCLCPP_INFO(node_->get_logger(),
                    "[calibrateCamera] Using previous random pos: (%.3f, %.3f, %.3f)",
                    randomPos.x, randomPos.y, randomPos.z);
    }

    // Compute orientation that points the +Z axis at (x,y,z) from randomPos
    geometry_msgs::msg::Quaternion q_look = lookAtOrientation(randomPos.x, randomPos.y, randomPos.z, x, y, z);

    // Create the pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position    = randomPos;
    target_pose.orientation = q_look;

    geometry_msgs::msg::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = "world";
    stamped_pose.header.stamp    = node_->now();
    stamped_pose.pose            = target_pose;

    if (!current_solver_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[calibrateCamera] No active solver set. Use choosePipeline() first.");
        executed_ = false;
        return;
    }

    // Build the MoveTo stage
    auto stage = std::make_unique<mtc::stages::MoveTo>("calibrateCamera random pose", current_solver_);
    stage->setGroup(arm_group_name_);
    stage->setTimeout(5.0);
    if (!tip_frame.empty())
        stage->setIKFrame(tip_frame);
    stage->setGoal(stamped_pose);

    task_.add(std::move(stage));

    RCLCPP_INFO(node_->get_logger(),
                "[calibrateCamera] Added random move to the Task. (No immediate execution.)");
    executed_ = true;
}

bool TaskBuilder::initTask()
{
    RCLCPP_INFO(node_->get_logger(), "[initTask] Initializing MTC Task");
    try {
        task_.init();
    } catch(const mtc::InitStageException& ex) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Task init failed: " << ex);
        executed_ = false;
        return false;
    }

    executed_ = true;
    return true;
}

bool TaskBuilder::planTask(unsigned max_solutions)
{
    RCLCPP_INFO(node_->get_logger(), "[planTask] Planning MTC Task");
    try {
        if (!task_.plan(max_solutions)) {
            RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
            executed_ = false;
            return false;
        }
        executed_ = true;
        return true;
    } catch(const mtc::InitStageException& ex) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Task planning threw: " << ex);
        executed_ = false;
        return false;
    }
}

bool TaskBuilder::executeTask()
{
    RCLCPP_INFO(node_->get_logger(), "[executeTask] Executing MTC Task");
    for (auto& solution : task_.solutions()) {
        task_.introspection().publishSolution(*solution);
        auto result = task_.execute(*solution);
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "Task execution failed!");
            executed_ = false;
            return false;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "Task execution SUCCESS!");
    executed_ = true;
    return true;
}

// ------------------------------------------------------------------------------------------
// The function below was previously doing a direct MoveGroup plan/execute; now it only builds
// MTC stages for each waypoint and defers actual execution to executeTask().
// ------------------------------------------------------------------------------------------
bool TaskBuilder::planWaypoints(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
                                 double velocity_scale,
                                 double accel_scale,
                                 double pose_tolerance)
{
    if (waypoints.empty()) {
        RCLCPP_WARN(node_->get_logger(), "[planWaypoints] No waypoints provided. Nothing to plan.");
        executed_ = false;
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[planWaypoints] Only building MTC stages. Execution must be done by executeTask().");
    RCLCPP_INFO(node_->get_logger(),
                "Velocity=%.2f, Accel=%.2f, Tolerance=%.2f, #Waypoints=%zu",
                velocity_scale, accel_scale, pose_tolerance, waypoints.size());

    // Optionally, set velocity/accel properties on the current solver
    if (current_solver_) {
        current_solver_->setProperty("max_velocity_scaling_factor", velocity_scale);
        current_solver_->setProperty("max_acceleration_scaling_factor", accel_scale);
    } else {
        RCLCPP_WARN(node_->get_logger(),
                    "[planWaypoints] No current_solver_ set. Using default scaling factors.");
    }

    // Add a MoveTo stage for each waypoint
    int index = 0;
    for (const auto& w : waypoints)
    {
        std::stringstream ss;
        ss << "waypoint_" << index++;

        auto stage = std::make_unique<mtc::stages::MoveTo>(ss.str(), current_solver_);
        stage->setGroup(arm_group_name_);
        stage->setGoal(w);
        // If you want to set the tip frame explicitly:
        stage->setIKFrame(tip_frame_);
        stage->setTimeout(5.0);
        if (solver_tolerance_) {
            stage->properties().set("goal_tolerance", solver_tolerance_);
        }
        task_.add(std::move(stage));
    }

    // We do NOT plan or execute here.  That must be done via:
    //   planTask();
    //   executeTask();
    executed_ = true;
    return true;
}
