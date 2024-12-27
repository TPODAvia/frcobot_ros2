#include <rclcpp/rclcpp.hpp>
#include "task_builder.hpp"
#include <iostream>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Expect at least 3 args: 
  //   argv[0] = "task_generator", 
  //   argv[1] = "fr10" (your robot name or similar),
  //   argv[2] = the command ("clear_scene", "remove_object", "joints_position", etc.)
  if (argc < 3) {
    std::cerr << "Usage: task_generator <robot> <command> [args...]" << std::endl;
    return 1;
  }

  std::string robot_name = argv[1];
  std::string command    = argv[2];

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("task_generator_node", node_options);

  TaskBuilder builder(node);
  // Create a new MTC Task named after the command (or your preference)
  builder.newTask("demo_task");

  // Dispatch the command
  if (command == "clear_scene") {
    builder.clearScene();

  } else if (command == "remove_object") {
    if (argc < 4) {
      std::cerr << "remove_object requires an object_name" << std::endl;
      return 1;
    }
    builder.removeObject(argv[3]);

  } else if (command == "joints_position") {
    // e.g. "joints_position 0 -1.57 1.57 0 0 0"
    if (argc < 9) {
      std::cerr << "Not enough joint values provided" << std::endl;
      return 1;
    }
    std::vector<double> joints;
    for (int i = 3; i < argc; ++i) {
      joints.push_back(std::stod(argv[i]));
    }
    builder.jointsPosition(joints);

  } else if (command == "spawn_object") {
    // e.g.: spawn_object hello_box 0 0.3 0.2 0.0 0.383 0.0 0.924
    if (argc < 11) {
      std::cerr << "Usage: spawn_object <name> <x> <y> <z> <rx> <ry> <rz> <rw>" << std::endl;
      return 1;
    }
    std::string obj_name = argv[3];
    double x  = std::stod(argv[4]);
    double y  = std::stod(argv[5]);
    double z  = std::stod(argv[6]);
    double rx = std::stod(argv[7]);
    double ry = std::stod(argv[8]);
    double rz = std::stod(argv[9]);
    double rw = std::stod(argv[10]);
    builder.spawnObject(obj_name, x, y, z, rx, ry, rz, rw);

  } else if (command == "choose_pipeline") {
    // e.g.: choose_pipeline OMPL RRTConnect
    if (argc < 5) {
      std::cerr << "Usage: choose_pipeline <pipeline_name> <planner_id>" << std::endl;
      return 1;
    }
    builder.choosePipeline(argv[3], argv[4]);

  } else if (command == "end_coordinate") {
    // e.g.: end_coordinate hello_box OR
    //       end_coordinate tf_end -0.33 0.63 0.11 -0.01 0.709 -0.01 0.705
    std::string frame_id = argv[3];
    if (argc == 4) {
      // no xyz, so just call with defaults
      builder.endCoordinate(frame_id);
    } else if (argc >= 11) {
      double x  = std::stod(argv[4]);
      double y  = std::stod(argv[5]);
      double z  = std::stod(argv[6]);
      double rx = std::stod(argv[7]);
      double ry = std::stod(argv[8]);
      double rz = std::stod(argv[9]);
      double rw = std::stod(argv[10]);
      builder.endCoordinate(frame_id, x, y, z, rx, ry, rz, rw);
    } else {
      std::cerr << "Usage: end_coordinate <frame_id> [x y z rx ry rz rw]" << std::endl;
      return 1;
    }

  } else if (command == "attach_object") {
    // e.g.: attach_object hello_box tf_end
    if (argc < 5) {
      std::cerr << "Usage: attach_object <object_name> <link_name>" << std::endl;
      return 1;
    }
    builder.attachObject(argv[3], argv[4]);

  } else if (command == "gripper_close") {
    builder.gripperClose();

  } else if (command == "gripper_open") {
    builder.gripperOpen();

  } else if (command == "detach_object") {
    // e.g.: detach_object hello_box tf_end
    if (argc < 5) {
      std::cerr << "Usage: detach_object <object_name> <link_name>" << std::endl;
      return 1;
    }
    builder.detachObject(argv[3], argv[4]);

  } else if (command == "delete_json_sim_content") {
    if (argc < 4) {
      std::cerr << "Usage: delete_json_sim_content <filename>" << std::endl;
      return 1;
    }
    builder.deleteJsonSimContent(argv[3]);

  } else {
    std::cerr << "Unknown command: " << command << std::endl;
    return 1;
  }

  // Now that we have added stages for the requested command(s),
  // we can plan & execute the task (if that is your design).
  // Typically, you'd do something like:
  if (!builder.initTask()) {
    return 1;
  }
  if (!builder.planTask()) {
    return 1;
  }
  if (!builder.executeTask()) {
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
