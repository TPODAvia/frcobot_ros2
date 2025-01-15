#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <map>
#include <algorithm>  // for std::remove_if
#include <cctype>     // for std::isspace
#include <geometry_msgs/msg/pose.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>
#include "task_builder.hpp"

enum class CommandKind {
  ROBOT_PARAM,
  CLEAR_SCENE,
  REMOVE_OBJECT,
  SPAWN_OBJECT,
  CHOOSE_PIPELINE,
  JOINTS_MOVE,
  ABSOLUTE_MOVE,
  DISPLACEMENT_MOVE,
  TRAJECTORY_MOVE,
  FEEDBACK_MOVE,
  COLLABORATIVE_MOVE,
  GRIPPER_CLOSE,
  GRIPPER_OPEN,
  ATTACH_OBJECT,
  DETACH_OBJECT,
  DELETE_JSON_SIM_CONTENT,
  DELETE_JSON_TEMP,
  CHECK_JSON_FILES,
  SCAN_LINE,
  CALIBRATE_CAMERA,
  GCODE_LOAD,
  STEP_LOAD,

  UNKNOWN
};

// Extend parseCommand to handle the 3 new commands
static CommandKind parseCommand(const std::string& cmd)
{
  if (cmd == "get_robot_param")           return CommandKind::ROBOT_PARAM;
  if (cmd == "clear_scene")               return CommandKind::CLEAR_SCENE;
  if (cmd == "remove_object")             return CommandKind::REMOVE_OBJECT;
  if (cmd == "spawn_object")              return CommandKind::SPAWN_OBJECT;
  if (cmd == "choose_pipeline")           return CommandKind::CHOOSE_PIPELINE;
  if (cmd == "joints_move")               return CommandKind::JOINTS_MOVE;
  if (cmd == "absolute_move")             return CommandKind::ABSOLUTE_MOVE;
  if (cmd == "displacement_move")         return CommandKind::DISPLACEMENT_MOVE;
  if (cmd == "trajectory_move")           return CommandKind::TRAJECTORY_MOVE;
  if (cmd == "feedback_move")             return CommandKind::FEEDBACK_MOVE;
  if (cmd == "collaborative_move")        return CommandKind::COLLABORATIVE_MOVE;
  if (cmd == "gripper_close")             return CommandKind::GRIPPER_CLOSE;
  if (cmd == "gripper_open")              return CommandKind::GRIPPER_OPEN;
  if (cmd == "attach_object")             return CommandKind::ATTACH_OBJECT;
  if (cmd == "detach_object")             return CommandKind::DETACH_OBJECT;
  if (cmd == "delete_json_sim_content")   return CommandKind::DELETE_JSON_SIM_CONTENT;
  if (cmd == "delete_json_temp")          return CommandKind::DELETE_JSON_TEMP;
  if (cmd == "check_json_files")          return CommandKind::CHECK_JSON_FILES;
  if (cmd == "scan_line")                 return CommandKind::SCAN_LINE;
  if (cmd == "calibrate_camera")          return CommandKind::CALIBRATE_CAMERA;
  if (cmd == "gcode_load")                return CommandKind::GCODE_LOAD;
  if (cmd == "step_load")                 return CommandKind::STEP_LOAD;

  return CommandKind::UNKNOWN;
}


static void removeUnwantedKeys(nlohmann::json& j, const std::vector<std::string>& keys_to_remove)
{
  if (j.is_object()) {
    // Erase unwanted keys from object
    for (auto it = j.begin(); it != j.end(); ) {
      // If the key is in the list, erase it
      if (std::find(keys_to_remove.begin(), keys_to_remove.end(), it.key()) != keys_to_remove.end()) {
        it = j.erase(it);
      } else {
        // Otherwise, recurse
        removeUnwantedKeys(it.value(), keys_to_remove);
        ++it;
      }
    }
  } else if (j.is_array()) {
    // Recurse for each element in the array
    for (auto& elem : j) {
      removeUnwantedKeys(elem, keys_to_remove);
    }
  }
}

/** 
 * CommandKind::DELETE_JSON_SIM_CONTENT 
 *
 * Usage (example):
 *   delete_json_sim_content <filename>
 *
 * Behavior:
 *   1. Check if <filename> exists; if not, print error and return.
 *   2. Remove "mod_<filename>" if it exists.
 *   3. Load JSON from <filename>.
 *   4. Remove unwanted keys: ["remove_object","detach_object","clear_scene","attach_object","spawn_object"].
 *   5. Filter out empty objects if they become empty {} in an array context.
 *   6. Save the cleaned JSON to "mod_<filename>".
 *   7. Print a success message and terminate.
 */
static int handleDeleteJsonSimContent(const std::string& filename, rclcpp::Node::SharedPtr node)
{
  // 1) Check if file exists
  if (!std::filesystem::exists(filename)) {
    RCLCPP_ERROR(node->get_logger(), "No file found at path: %s", filename.c_str());
    return 1;
  }

  // 2) Build the mod_ filename in the same directory
  std::filesystem::path p(filename);
  std::filesystem::path mod_file = p.parent_path() / ("mod_" + p.filename().string());

  // If it already exists, remove it
  if (std::filesystem::exists(mod_file)) {
    std::filesystem::remove(mod_file);
    RCLCPP_INFO(node->get_logger(), "Removed existing mod-file: %s", mod_file.string().c_str());
  }

  // 3) Load JSON from <filename>
  nlohmann::json data;
  {
    std::ifstream fin(filename);
    if (!fin.is_open()) {
      RCLCPP_ERROR(node->get_logger(), "Failed to open JSON file: %s", filename.c_str());
      return 1;
    }
    try {
      fin >> data;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), "Error parsing JSON: %s", e.what());
      return 1;
    }
  }

  // 4) Remove unwanted keys
  std::vector<std::string> unwanted_keys = {
    "remove_object", "detach_object", "clear_scene", "attach_object", "spawn_object"
  };
  removeUnwantedKeys(data, unwanted_keys);

  // 5) Filter out empty dictionaries ({}), if JSON is an array
  //    so we don't keep empty objects in the top-level array
  if (data.is_array()) {
    nlohmann::json filtered = nlohmann::json::array();
    for (auto& item : data) {
      // keep item if it's not an empty object
      if (!item.is_object() || !item.empty()) {
        filtered.push_back(item);
      }
    }
    data = filtered;
  }

  // 6) Save the cleaned JSON to "mod_<filename>"
  {
    std::ofstream fout(mod_file);
    if (!fout.is_open()) {
      RCLCPP_ERROR(node->get_logger(), "Failed to open output file: %s", mod_file.string().c_str());
      return 1;
    }
    fout << data.dump(2) << std::endl;
  }

  RCLCPP_INFO(node->get_logger(), "delete_json_sim_content finished.");
  return 0;
}

/**
 * CommandKind::CHECK_JSON_FILES
 *
 * Usage (example):
 *   check_json_files <directory>
 *
 * Behavior:
 *   1. List all `.json` files in <directory>.
 *   2. Attempt to parse each as JSON.
 *   3. Print which files are valid/invalid.
 *   4. Advise manual checks if desired.
 */
static int handleCheckJsonFiles(const std::string& directory, rclcpp::Node::SharedPtr node)
{
  if (!std::filesystem::exists(directory) || !std::filesystem::is_directory(directory)) {
    RCLCPP_ERROR(node->get_logger(), "Invalid directory: %s", directory.c_str());
    return 1;
  }

  std::vector<std::string> valid_json_files;
  std::vector<std::string> invalid_json_files;

  for (auto& entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_regular_file()) {
      std::filesystem::path fpath = entry.path();
      if (fpath.extension() == ".json") {
        // Attempt to parse
        std::ifstream fin(fpath.string());
        if (!fin.is_open()) {
          invalid_json_files.push_back(fpath.filename().string());
          continue;
        }
        try {
          nlohmann::json data;
          fin >> data;  // If parse fails, it will throw
          valid_json_files.push_back(fpath.filename().string());
        } catch (const std::exception& e) {
          invalid_json_files.push_back(fpath.filename().string());
          RCLCPP_WARN(node->get_logger(),
            "Invalid JSON in file: %s - Error: %s",
            fpath.filename().string().c_str(),
            e.what());
        }
      }
    }
  }

  // Print summary
  std::cout << std::string(80, '#') << "\n";
  std::cout << "Directory: " << directory << "\n";
  std::cout << std::string(80, '#') << "\n\n";

  std::cout << "Valid JSON files:\n";
  for (auto& f : valid_json_files) {
    std::cout << "  " << f << "\n";
  }
  std::cout << "\nInvalid JSON files:\n";
  for (auto& f : invalid_json_files) {
    std::cout << "  " << f << "\n";
  }
  std::cout << std::string(80, '#') << "\n";
  std::cout << "WARNING: This only checks JSON syntax. Further manual checks may be necessary.\n";
  std::cout << "check_json_files finished!\n";

  return 0;
}

/**
 * CommandKind::DELETE_JSON_TEMP
 *
 * Usage (example):
 *   delete_json_temp <directory>
 *
 * Behavior:
 *   1. Ask for user confirmation ("y/n").
 *   2. If "y", remove `test.json` and `mod_test.json` in <directory> if they exist.
 *   3. Otherwise, do nothing.
 */
static int handleDeleteJsonTemp(const std::string& directory, rclcpp::Node::SharedPtr node)
{
  std::cout << "test.json and mod_test.json will be deleted in: " << directory << "\n";
  std::cout << "Proceed? (y/n): ";
  std::string answer;
  std::cin >> answer;

  if (answer == "y" || answer == "Y") {
    std::filesystem::path test_file      = std::filesystem::path(directory) / "test.json";
    std::filesystem::path mod_test_file  = std::filesystem::path(directory) / "mod_test.json";

    if (std::filesystem::exists(test_file)) {
      std::filesystem::remove(test_file);
      RCLCPP_INFO(node->get_logger(), "Removed: %s", test_file.string().c_str());
    }
    if (std::filesystem::exists(mod_test_file)) {
      std::filesystem::remove(mod_test_file);
      RCLCPP_INFO(node->get_logger(), "Removed: %s", mod_test_file.string().c_str());
    }
  } else {
    std::cout << "Deletion cancelled. Files are not deleted.\n";
  }

  RCLCPP_INFO(node->get_logger(), "delete_json_temp finished.");
  return 0;
}

static void savePosesToFile(const std::string& filename, const std::vector<geometry_msgs::msg::Pose>& poses) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Failed to open file for saving poses: " << filename << std::endl;
    return;
  }

  file << "Saved Poses (" << poses.size() << " poses):" << std::endl;
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto& pose = poses[i];
    file << "Pose #" << i + 1 << ":" << std::endl;
    file << "  Position: (" 
         << pose.position.x << ", " 
         << pose.position.y << ", " 
         << pose.position.z << ")" << std::endl;
    file << "  Orientation: (" 
         << pose.orientation.x << ", " 
         << pose.orientation.y << ", " 
         << pose.orientation.z << ", " 
         << pose.orientation.w << ")" << std::endl;
  }

  file.close();
  std::cout << "Poses successfully saved to " << filename << std::endl;
}

/**
 * Example helper function to linearly interpolate between two poses.
 * You might want a more sophisticated interpolation (e.g., SLERP for orientation).
 */
static geometry_msgs::msg::Pose interpolatePose(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& end,
    double t)
{
  geometry_msgs::msg::Pose result;
  // Linear interpolation for position
  result.position.x = start.position.x + t * (end.position.x - start.position.x);
  result.position.y = start.position.y + t * (end.position.y - start.position.y);
  result.position.z = start.position.z + t * (end.position.z - start.position.z);

  // Linear interpolation for orientation (not SLERP, just naive approach)
  result.orientation.x = start.orientation.x + t * (end.orientation.x - start.orientation.x);
  result.orientation.y = start.orientation.y + t * (end.orientation.y - start.orientation.y);
  result.orientation.z = start.orientation.z + t * (end.orientation.z - start.orientation.z);
  result.orientation.w = start.orientation.w + t * (end.orientation.w - start.orientation.w);

  return result;
}

/**
 * A simple structure representing a single parsed G-code command.
 *
 * Examples:
 *  - original_line: "G1 X10.0 Y20.0 Z5.0 F800 ; move quickly"
 *  - code: "G1"
 *  - params: { {'X', 10.0}, {'Y', 20.0}, {'Z', 5.0}, {'F', 800.0} }
 *  - comment: "move quickly"
 */
struct GCodeCommand
{
  std::string original_line;        // The full, raw line (minus trailing whitespace).
  std::string code;                 // E.g. "G0", "G1", "G2", "M104", "T0", ...
  std::map<char, double> params;    // Parameter letter -> numeric value (X->10, Y->20, Z->5, F->800, etc)
  std::string comment;              // Any trailing comment after ';' or '('...')', etc.
};

/**
 * @brief Extract the G-code command token (e.g. "G1", "M104", "T0") from the beginning of a line
 *        or from the first token. Anything that starts with 'G', 'M', or 'T' (followed by digits)
 *        is handled here. If no command is found, returns an empty string.
 */
static std::string extractCommandToken(const std::string& token)
{
  // Typical G-code commands can start with G / M / T (or others). Examples:
  //  "G1", "M104", "T1", "M106", "G28.1", etc.
  // We do a simple check: if the first character is G/M/T (upper or lower),
  // followed by optional digits or a decimal point, we treat it as a command.
  if (token.empty())
    return "";

  char c = std::toupper(token[0]);
  if (c == 'G' || c == 'M' || c == 'T') {
    // We'll accept the entire token as the "code": e.g. "G1", "M104", "T0"
    // If you want stricter parsing, you can limit it to letters + digits
    return token;
  }

  return "";
}

/**
 * @brief Parse one line of G-code into a GCodeCommand.
 *
 * This function:
 *  1. Strips out any comment (semicolon `;` or parenthetical `( ... )`).
 *  2. Extracts the first token if it is a command (e.g., "G1", "M104", etc).
 *  3. For each subsequent token like "X10.0" or "F800", extracts the letter
 *     and numeric value. Non-numeric tokens are ignored.
 *  4. Stores the results in a GCodeCommand struct.
 */
static GCodeCommand parseGCodeLine(const std::string& raw_line)
{
  GCodeCommand cmd;
  cmd.original_line = raw_line;

  // 1) Remove inline comments starting with ';'
  //    (Also handle parentheses comments if you wish.)
  //    For simplicity, let's just cut everything from the first ';' onward.
  std::string line = raw_line;
  auto sc_pos = line.find(';');
  if (sc_pos != std::string::npos) {
    // The part after ';' is considered a comment
    cmd.comment = line.substr(sc_pos + 1);
    line = line.substr(0, sc_pos);
  }

  // Optional: also handle '(' ... ')' comments if needed:
  //   This can be more elaborate if multiple parentheses can appear.
  //   We'll do a naive single pass to remove them:
  auto openParen = line.find('(');
  auto closeParen = line.find(')', openParen + 1);
  if (openParen != std::string::npos && closeParen != std::string::npos) {
    // Everything inside is comment
    std::string parenComment = line.substr(openParen + 1, (closeParen - openParen - 1));
    if (!cmd.comment.empty()) cmd.comment += " | ";
    cmd.comment += parenComment;
    // Remove that portion from the line
    line.erase(openParen, (closeParen - openParen + 1));
  }

  // Trim trailing/leading spaces
  auto trimSpace = [](std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            [](unsigned char ch){ return !std::isspace(ch); }));
    s.erase(std::find_if(s.rbegin(), s.rend(),
            [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end());
  };
  trimSpace(line);

  // 2) Tokenize what's left
  std::stringstream ss(line);
  std::string token;
  bool firstTokenProcessed = false;
  while (ss >> token) {
    // If this is the first token, see if it's a G/M/T command
    if (!firstTokenProcessed) {
      std::string possibleCmd = extractCommandToken(token);
      if (!possibleCmd.empty()) {
        cmd.code = possibleCmd;    // e.g. "G1"
        firstTokenProcessed = true;
        continue;
      }
      // If it's not recognized as G/M/T, we don't treat it as "cmd".
      // We'll still parse parameters from it if it looks like X10, Y20, etc.
      firstTokenProcessed = true; // we skip command
    }

    // Check if the token is something like "X10.0" or "E2.5"
    // The first letter is the parameter key, the remainder is numeric
    if (token.size() >= 2) {
      char paramLetter = std::toupper(token[0]);
      std::string valStr = token.substr(1);
      // Attempt to parse the remainder as a double
      try {
        double value = std::stod(valStr);
        cmd.params[paramLetter] = value;
      } catch(...) {
        // If it fails, ignore this token or handle error
      }
    }
  }

  return cmd;
}

/**
 * @brief Main G-code parser function. 
 * 
 * Returns a vector of geometry_msgs::msg::Pose that only includes positions 
 * for linear G0/G1 moves. All commands (G, M, T, etc.) are parsed and printed 
 * out or could be further processed (see 'all_commands').
 */
static std::vector<geometry_msgs::msg::Pose> parseGCodeFile(const std::string& filename)
{
  // We'll keep track of all parsed commands
  std::vector<GCodeCommand> all_commands;

  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Failed to open G-code file: " << filename << std::endl;
    return {};
  }

  // Current position state (for G0/G1). 
  // We'll track X/Y/Z (and optionally E if 3D printing).
  double x = 0.0, y = 0.0, z = 0.0;

  std::string raw_line;
  while (std::getline(file, raw_line)) {
    if (raw_line.empty()) 
      continue;

    // 1) Parse the line into a GCodeCommand struct
    GCodeCommand cmd = parseGCodeLine(raw_line);
    if (!cmd.code.empty() || !cmd.params.empty()) {
      all_commands.push_back(cmd);
    }
  }
  file.close();

  // 2) Now convert G0/G1 moves into a path of geometry_msgs::msg::Pose
  std::vector<geometry_msgs::msg::Pose> path;

  for (auto& c : all_commands) {
    // For demonstration, only handle G0 or G1 as linear moves
    if (c.code == "G0" || c.code == "G1") {
      // If X is present, update it
      if (c.params.find('X') != c.params.end()) {
        x = c.params.at('X');
      }
      // If Y is present, update it
      if (c.params.find('Y') != c.params.end()) {
        y = c.params.at('Y');
      }
      // If Z is present, update it
      if (c.params.find('Z') != c.params.end()) {
        z = c.params.at('Z');
      }

      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      // keep orientation fixed for demonstration
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;

      path.push_back(pose);
    }
  }

  // [Optional] Print out or debug all commands
  // for (auto& c : all_commands) {
  //   std::cout << "[CMD] " << c.original_line << "\n"
  //             << "     Code: " << c.code
  //             << " | Params: ";
  //   for (auto& kv : c.params) {
  //     std::cout << kv.first << "=" << kv.second << " ";
  //   }
  //   std::cout << "\n     Comment: " << c.comment << "\n\n";
  // }

  return path; 
}

static std::vector<geometry_msgs::msg::Pose> parseStepFile(const std::string& filename)
{
  std::vector<geometry_msgs::msg::Pose> result;

  // 1) Open the file
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "parseStepFile: could not open file: " << filename << std::endl;
    return result;
  }

  // 2) Read all lines into memory
  std::vector<std::string> lines;
  {
    std::string line;
    while (std::getline(file, line)) {
      lines.push_back(line);
    }
  }
  file.close();

  // 3) Identify a B-spline definition line (first occurrence)
  //    e.g. #47=B_SPLINE_CURVE_WITH_KNOTS(...)
  std::string spline_ref;
  for (const auto &l : lines) {
    if (l.find("B_SPLINE_CURVE_WITH_KNOTS") != std::string::npos) {
      spline_ref = l;
      break;
    }
  }

  if (spline_ref.empty()) {
    std::cerr << "No B_SPLINE_CURVE_WITH_KNOTS found in " << filename << std::endl;
    return result;
  }

  // 4) From that line, extract references to CARTESIAN_POINTs 
  //    e.g. ( #59, #60, #61, ... )
  std::vector<std::string> cartesian_refs;
  {
    auto startPos = spline_ref.find("(#");
    if (startPos == std::string::npos) {
      std::cerr << "Could not parse control points list." << std::endl;
      return result;
    }
    // find the matching closing parenthesis for that "("
    auto endPos = spline_ref.find(")", startPos);
    if (endPos == std::string::npos) {
      std::cerr << "Could not parse control points list (no closing parenthesis)." << std::endl;
      return result;
    }

    // substring of "#59,#60,#61,..."
    std::string refs = spline_ref.substr(startPos, endPos - startPos);

    // remove parentheses
    refs.erase(std::remove(refs.begin(), refs.end(), '('), refs.end());
    refs.erase(std::remove(refs.begin(), refs.end(), ')'), refs.end());

    // now something like "#59,#60,#61,#62,#63..."
    std::stringstream ss(refs);
    std::string token;
    while (std::getline(ss, token, ',')) {
      // Look for tokens that contain '#' (e.g. "#59")
      if (!token.empty() && token.find("#") != std::string::npos) {
        // trim leading/trailing spaces
        while (!token.empty() && std::isspace(token.front())) token.erase(token.begin());
        while (!token.empty() && std::isspace(token.back()))  token.pop_back();

        cartesian_refs.push_back(token);
      }
    }
  }

  if (cartesian_refs.empty()) {
    std::cerr << "No cartesian point references found." << std::endl;
    return result;
  }

  // 5) For each reference #xx, find the line with CARTESIAN_POINT(...) and parse X,Y,Z
  struct Point3 { double x, y, z; };
  std::vector<Point3> control_points;

  for (const auto &ref : cartesian_refs) {
    for (const auto &l : lines) {
      // Example line:
      // #59=CARTESIAN_POINT('',(-61.58,-48.59,42.20));
      // We'll look for "ref=" at the start AND "CARTESIAN_POINT"
      if (l.rfind(ref + "=", 0) == 0 && l.find("CARTESIAN_POINT") != std::string::npos) {
        // Find the substring for the second parentheses, which actually has (x,y,z).
        // The line might look like: CARTESIAN_POINT('',(-61.58,-48.59,42.20))
        // We skip the first '(' after CARTESIAN_POINT( to handle `'...'`.
        auto cpointPos = l.find("CARTESIAN_POINT(");
        if (cpointPos == std::string::npos) 
          continue;

        cpointPos += std::strlen("CARTESIAN_POINT("); // move past "CARTESIAN_POINT("

        // Now find the second '(' that starts x,y,z
        auto secondParen = l.find("(", cpointPos);
        if (secondParen == std::string::npos)
          continue;

        // Find the matching ')'
        auto endParen = l.find(")", secondParen + 1);
        if (endParen == std::string::npos)
          continue;

        // Extract the coordinate substring
        std::string coords = l.substr(secondParen + 1, endParen - (secondParen + 1));
        // e.g. "-61.58,-48.59,42.20"
        std::stringstream ssc(coords);
        std::string val;
        std::vector<double> vals;

        while (std::getline(ssc, val, ',')) {
          // trim spaces
          while (!val.empty() && std::isspace(val.front())) val.erase(val.begin());
          while (!val.empty() && std::isspace(val.back()))  val.pop_back();

          vals.push_back(std::stod(val));
        }

        if (vals.size() == 3) {
          control_points.push_back({vals[0], vals[1], vals[2]});
        }
        break; // Stop searching lines once found
      }
    }
  }

  // 6) Minimal check: do we have enough control points to form a curve?
  if (control_points.size() < 2) {
    std::cerr << "Not enough control points to form a curve." << std::endl;
    return result;
  }

  // 7) Naive approach: sample between control points uniformly
  int num_samples = 20;
  for (int i = 0; i < num_samples; ++i) {
    double t = static_cast<double>(i) / (num_samples - 1); // range [0..1]
    double float_index = t * (control_points.size() - 1);

    int idx0 = static_cast<int>(std::floor(float_index));
    int idx1 = std::min(idx0 + 1, static_cast<int>(control_points.size() - 1));
    double ratio = float_index - static_cast<double>(idx0);

    double x = control_points[idx0].x + ratio * (control_points[idx1].x - control_points[idx0].x);
    double y = control_points[idx0].y + ratio * (control_points[idx1].y - control_points[idx0].y);
    double z = control_points[idx0].z + ratio * (control_points[idx1].z - control_points[idx0].z);

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    // Identity orientation; adjust as needed
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    result.push_back(pose);
  }

  return result;
}

static bool isNumeric(const std::string& s)
{
  try {
    /* Attempt to parse as a double */
    std::size_t pos;
    std::stod(s, &pos);  // may throw
    // If pos != s.size(), there is extra non-numeric text
    if (pos != s.size()) 
      return false;
    return true;
  } catch (...) {
    return false; 
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("task_generator_node", options);

  std::cout << "argc: " << argc << std::endl;
  std::cout << "argv values:" << std::endl;
  for (int i = 0; i < argc; ++i) {
    std::cout << "  argv[" << i << "]: " << argv[i] << std::endl;
  }

  // [task_generator-1] argc: 7
  // [task_generator-1] argv values:
  // [task_generator-1]   argv[0]: /home/vboxuser/colcon_ws/install/fairino_mtc_demo/lib/fairino_mtc_demo/task_generator
  // [task_generator-1]   argv[1]: fairino10_v6_group
  // [task_generator-1]   argv[4]: --ros-args
  // [task_generator-1]   argv[5]: -r
  // [task_generator-1]   argv[6]: __node:=task_generator_node
  // [task_generator-1]   argv[7]: --params-file
  // [task_generator-1]   argv[8]: /tmp/launch_params_i7ymn_yw

  if (argc < 15) {
    RCLCPP_ERROR(node->get_logger(),
                 "Error config. Usage: task_generator <arm_group_name> <tip_frame> <command> [optional: args...]");
    rclcpp::shutdown();
    return 1;
  }

  // Extract arguments
  std::string arm_group_name = argv[1];
  std::string tip_frame      = argv[2];
  bool exec_task             = argv[3];
  bool save_json             = argv[4];
  bool reserved_1            = argv[5];
  bool reserved_2            = argv[6];
  bool reserved_3            = argv[7];
  bool reserved_4            = argv[8];
  std::string command_str    = argv[9];

  // Suppress unused variable warnings
  (void)reserved_1;
  (void)reserved_2;
  (void)reserved_3;
  (void)reserved_4;

  // if (!node->has_parameter("robot_description")) {
  //   RCLCPP_WARN(node->get_logger(),
  //               "Parameter 'robot_description' not found! Did you load the MoveIt config?");
  // } else {
  //   std::string urdf_string = node->get_parameter("robot_description").as_string();
  //   RCLCPP_INFO(node->get_logger(), "Loaded robot_description: %zu characters", urdf_string.size());
  // }

  TaskBuilder builder(node, arm_group_name, tip_frame);
  builder.newTask("demo_task");

  // Switch on the parsed command
  switch (parseCommand(command_str))
  {
    case CommandKind::ROBOT_PARAM:
    {
      builder.printRobotParams();
      break;
    }

    case CommandKind::CLEAR_SCENE:
    {
      builder.clearScene();
      break;
    }

    case CommandKind::REMOVE_OBJECT:
    {
      // "remove_object <object_name>"
      if (argc != 16) {
        RCLCPP_ERROR(node->get_logger(),
                     "remove_object requires an object_name (remove_object <object_name>)");
        rclcpp::shutdown();
        return 1;
      }
      builder.removeObject(argv[10]);
      break;
    }

    case CommandKind::SPAWN_OBJECT:
    {
      // spawn_object object 1 1 1 0 0 0 1
      if (argc != 26) {
        RCLCPP_ERROR(node->get_logger(),
          "Usage: spawn_object <obj_name> <x> <y> <z> <rx> <ry> <rz> <rw> <da> <db> <dc>");
        rclcpp::shutdown();
        return 1;
      }
      std::string obj_name = argv[10];
      double x  = std::stod(argv[11]);
      double y  = std::stod(argv[12]);
      double z  = std::stod(argv[13]);
      double rx = std::stod(argv[14]);
      double ry = std::stod(argv[15]);
      double rz = std::stod(argv[16]);
      double rw = std::stod(argv[17]);
      double da = std::stod(argv[18]);
      double db = std::stod(argv[19]);
      double dc = std::stod(argv[20]);
      builder.spawnObject(obj_name, obj_name, x, y, z, rx, ry, rz, rw, da, db, dc);
      break;
    }

    case CommandKind::CHOOSE_PIPELINE:
    {
      // choose_pipeline <pipeline_name> <planner_id>
      if (argc < 14) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: choose_pipeline <pipeline_name> <planner_id>");
        rclcpp::shutdown();
        return 1;
      }
      builder.choosePipeline(argv[10], argv[11]);
      break;
    }

    case CommandKind::JOINTS_MOVE:
    {
      // joints_move <j1> <j2> <j3> <j4> <j5> <j6>
      if (argc < 15) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: joints_move <j1> <j2> <j3> <j4> <j5> <j6>");
        rclcpp::shutdown();
        return 1;
      }
      std::vector<double> joint_values;
      if (argc != 15) {
        for (int i = 10; i < 16; ++i) {
          joint_values.push_back(std::stod(argv[i]));
        }
        
      }
      builder.jointsMove(joint_values);
      break;
    }

    case CommandKind::ABSOLUTE_MOVE:
    {
      // absolute_move <frame_id> [tip_frame target_frame] OR <frame_id> [x y z rx ry rz rw]
      if (argc < 11) {
        RCLCPP_ERROR(node->get_logger(),
                    "absolute_move requires at least a frame_id and either (tip_frame, target_frame) or (x, y, z, rx, ry, rz, rw)");
        rclcpp::shutdown();
        return 1;
      }

      std::string frame_id = argv[10];  // e.g., "world" or "base_link"

      // -----------------------------------------------------------------
      // Case 1: absolute_move <frame_id> <tip_frame> <target_frame>
      if (argc == 18) {
        // Check if argv[10] to argv[12] are non-numeric strings
        if (isNumeric(argv[10]) || isNumeric(argv[11]) || isNumeric(argv[12])) {
          RCLCPP_ERROR(node->get_logger(),
                      "absolute_move: expected tip_frame and target_frame as non-numeric strings");
          rclcpp::shutdown();
          return 1;
        }

        std::string tip_frame = argv[11];
        std::string target_frame = argv[12];

        // Call the builder's absoluteMove with frame IDs only (no pose specified)
        builder.absoluteMove(frame_id, tip_frame, target_frame);
      }
      // -----------------------------------------------------------------
      // Case 2: absolute_move <frame_id> [x y z rx ry rz rw]
      else if (argc == 23) {
        // Check if argv[11] to argv[17] are all numeric values
        bool numeric_args = true;
        for (int i = 11; i <= 17; ++i) {
          if (!isNumeric(argv[i])) {
            numeric_args = false;
            break;
          }
        }

        if (!numeric_args) {
          RCLCPP_ERROR(node->get_logger(),
                      "absolute_move: expected 7 numeric arguments for x, y, z, rx, ry, rz, rw");
          rclcpp::shutdown();
          return 1;
        }

        double x  = std::stod(argv[11]);
        double y  = std::stod(argv[12]);
        double z  = std::stod(argv[13]);
        double rx = std::stod(argv[14]);
        double ry = std::stod(argv[15]);
        double rz = std::stod(argv[16]);
        double rw = std::stod(argv[17]);

        // Call the builder's absoluteMove with the specified pose
        builder.absoluteMove(frame_id, "", "", x, y, z, rx, ry, rz, rw);
      }
      // -----------------------------------------------------------------
      // Case 3: Incorrect usage
      else {
        RCLCPP_ERROR(node->get_logger(),
                    "Usage: absolute_move <frame_id> [tip_frame target_frame] OR <frame_id> [x y z rx ry rz rw]");
        rclcpp::shutdown();
        return 1;
      }

      break;
    }

    case CommandKind::DISPLACEMENT_MOVE:
    {
      // Check if argv[10] and argv[11] are non-numeric strings
      if (isNumeric(argv[10]) || isNumeric(argv[11])) {
        RCLCPP_ERROR(node->get_logger(),
                    "absolute_move: expected tip_frame and target_frame as non-numeric strings");
        rclcpp::shutdown();
        return 1;
      }
      // displacement_move <world_frame> <tip_frame> <x> <y> <z> <rx> <ry> <rz>
      if (argc != 23) {
        RCLCPP_ERROR(node->get_logger(),
                    "Usage: displacement_move <world_frame> <tip_frame> <x> <y> <z> <rx> <ry> <rz>");
        rclcpp::shutdown();
        return 1;
      }


      std::string world_frame = argv[10];
      std::string tip_frame = argv[11];
      
      // Extract translation vector (x, y, z)
      std::vector<double> translation_vector;
      for (int i = 12; i < 15; ++i) {
        translation_vector.push_back(std::stod(argv[i]));
      }

      // Extract rotation (rx, ry, rz, rw)
      std::vector<double> rotation_vector;
      for (int i = 15; i < 18; ++i) {
        rotation_vector.push_back(std::stod(argv[i]));
      }

      // Call the modified displacementMove method
      builder.displacementMove(world_frame, tip_frame, translation_vector, rotation_vector);
      break;
    }

    case CommandKind::TRAJECTORY_MOVE:
    {
      // Option 1) user calls:
      //   trajectory_move p1_x p1_y p1_z p1_rx p1_ry p1_rz p1_rw ...
      //
      // Option 2) user calls (with --use-vel):
      //   trajectory_move --use-vel p1_x p1_y p1_z p1_rx p1_ry p1_rz p1_rw p1_vx p1_vy p1_vz ...
      //
      // Decide which parsing approach to use
      bool use_velocities = false;
      int start_index = 10; 
      if (std::string(argv[10]) == "--use-vel") {
        use_velocities = true;
        start_index = 11;
      }

      if (!use_velocities)
      {
        // ------- Original positions-only parsing -------
        // Must have multiples of 7 from `start_index`
        if (((argc - start_index) % 7 != 0) || (argc - start_index) < 7) {
          RCLCPP_ERROR(node->get_logger(),
                      "Usage (positions only): trajectory_move [--use-vel] <p1_x> <p1_y> <p1_z> <p1_rx> <p1_ry> <p1_rz> <p1_rw> ...");
          rclcpp::shutdown();
          return 1;
        }
        std::vector<geometry_msgs::msg::Pose> trajectory;
        int num_poses = (argc - start_index) / 7;
        for (int p = 0; p < num_poses; ++p) {
          geometry_msgs::msg::Pose pose;
          pose.position.x    = std::stod(argv[start_index + p * 7 + 0]);
          pose.position.y    = std::stod(argv[start_index + p * 7 + 1]);
          pose.position.z    = std::stod(argv[start_index + p * 7 + 2]);
          pose.orientation.x = std::stod(argv[start_index + p * 7 + 3]);
          pose.orientation.y = std::stod(argv[start_index + p * 7 + 4]);
          pose.orientation.z = std::stod(argv[start_index + p * 7 + 5]);
          pose.orientation.w = std::stod(argv[start_index + p * 7 + 6]);
          trajectory.push_back(pose);
        }
        // Original function call
        builder.trajectoryMove(trajectory);
      }
      else
      {
        // ------- Positions + velocities parsing -------
        // We expect 13 values per waypoint: 
        // 7 for pose + 6 for velocity (vx, vy, vz, wx, wy, wz)
        // Adjust to your needs (some prefer linear + angular speeds or something else).
        if (((argc - start_index) % 13 != 0) || (argc - start_index) < 13) {
          RCLCPP_ERROR(node->get_logger(),
                      "Usage (with velocities): trajectory_move --use-vel <p1_x> ... <p1_w> <p1_vx> <p1_vy> <p1_vz> <p1_wx> <p1_wy> <p1_wz> ...");
          rclcpp::shutdown();
          return 1;
        }
        int num_waypoints = (argc - start_index) / 13;
        std::vector<geometry_msgs::msg::Pose>   poses(num_waypoints);
        std::vector<geometry_msgs::msg::Twist>  velocities(num_waypoints);

        for (int i = 0; i < num_waypoints; ++i) {
          int base = start_index + i * 13;
          poses[i].position.x    = std::stod(argv[base + 0]);
          poses[i].position.y    = std::stod(argv[base + 1]);
          poses[i].position.z    = std::stod(argv[base + 2]);
          poses[i].orientation.x = std::stod(argv[base + 3]);
          poses[i].orientation.y = std::stod(argv[base + 4]);
          poses[i].orientation.z = std::stod(argv[base + 5]);
          poses[i].orientation.w = std::stod(argv[base + 6]);

          velocities[i].linear.x  = std::stod(argv[base + 7]);
          velocities[i].linear.y  = std::stod(argv[base + 8]);
          velocities[i].linear.z  = std::stod(argv[base + 9]);
          velocities[i].angular.x = std::stod(argv[base + 10]);
          velocities[i].angular.y = std::stod(argv[base + 11]);
          velocities[i].angular.z = std::stod(argv[base + 12]);
        }

        // Call the new variant that accepts velocities
        builder.trajectoryMoveV(poses, velocities);
      }
      break;
    }

    case CommandKind::FEEDBACK_MOVE:
    {
      // feedback_move
      builder.feedbackMove();
      break;
    }

    case CommandKind::COLLABORATIVE_MOVE:
    {
      // 
      // builder.feedbackMove();
      break;
    }

    case CommandKind::GRIPPER_CLOSE:
    {
      builder.gripperClose();
      break;
    }

    case CommandKind::GRIPPER_OPEN:
    {
      builder.gripperOpen();
      break;
    }

    case CommandKind::ATTACH_OBJECT:
    {
      // attach_object <object_name> <link_name>
      if (argc != 17) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: attach_object <object_name> <link_name>");
        rclcpp::shutdown();
        return 1;
      }
      builder.attachObject(argv[10], argv[11]);
      break;
    }

    case CommandKind::DETACH_OBJECT:
    {
      // detach_object <object_name> <link_name>
      if (argc != 17) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: detach_object <object_name> <link_name>");
        rclcpp::shutdown();
        return 1;
      }
      builder.detachObject(argv[10], argv[11]);
      break;
    }

    case CommandKind::DELETE_JSON_SIM_CONTENT:
    {
      // delete_json_sim_content <filename>
      if (argc != 16) {
        RCLCPP_ERROR(node->get_logger(),
                    "Usage: delete_json_sim_content <filename>");
        rclcpp::shutdown();
        return 1;
      }

      std::string filename = argv[10];
      int rc = handleDeleteJsonSimContent(filename, node);
      rclcpp::shutdown();
      return rc;
    }

    case CommandKind::CHECK_JSON_FILES:
    {
      // check_json_files <directory>
      if (argc != 16) {
        RCLCPP_ERROR(node->get_logger(),
                    "Usage: check_json_files <directory>");
        rclcpp::shutdown();
        return 1;
      }

      std::string directory = argv[10];
      int rc = handleCheckJsonFiles(directory, node);
      rclcpp::shutdown();
      return rc;
    }

    case CommandKind::DELETE_JSON_TEMP:
    {
      // delete_json_temp <directory>
      if (argc != 16) {
        RCLCPP_ERROR(node->get_logger(),
                    "Usage: delete_json_temp <directory>");
        rclcpp::shutdown();
        return 1;
      }

      std::string directory = argv[10];
      int rc = handleDeleteJsonTemp(directory, node);
      rclcpp::shutdown();
      return rc;
    }

    case CommandKind::SCAN_LINE:
    {
      // Example usage:
      //   scan_line <start_x> <start_y> <start_z> <start_rx> <start_ry> <start_rz> <start_rw>
      //             <end_x>   <end_y>   <end_z>   <end_rx>   <end_ry>   <end_rz>   <end_rw>
      //             <num_steps>
      // That is 1 command + 14 pose values + 1 integer => 16 arguments after <arm_group_name> <tip_frame> <command>
      if (argc < 19) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: scan_line <sx> <sy> <sz> <srx> <sry> <srz> <srw> <ex> <ey> <ez> <erx> <ery> <erz> <erw> <num_steps>");
        rclcpp::shutdown();
        return 1;
      }
      geometry_msgs::msg::Pose start_pose, end_pose;
      start_pose.position.x = std::stod(argv[4]);
      start_pose.position.y = std::stod(argv[5]);
      start_pose.position.z = std::stod(argv[6]);
      start_pose.orientation.x = std::stod(argv[7]);
      start_pose.orientation.y = std::stod(argv[8]);
      start_pose.orientation.z = std::stod(argv[9]);
      start_pose.orientation.w = std::stod(argv[10]);

      end_pose.position.x = std::stod(argv[11]);
      end_pose.position.y = std::stod(argv[12]);
      end_pose.position.z = std::stod(argv[13]);
      end_pose.orientation.x = std::stod(argv[14]);
      end_pose.orientation.y = std::stod(argv[15]);
      end_pose.orientation.z = std::stod(argv[16]);
      end_pose.orientation.w = std::stod(argv[17]);

      int num_steps = std::stoi(argv[18]);
      if (num_steps < 2) num_steps = 2; // at least 2 points

      // Build a trajectory from start to end
      std::vector<geometry_msgs::msg::Pose> scan_trajectory;
      scan_trajectory.reserve(num_steps);

      for (int i = 0; i < num_steps; ++i) {
        double t = static_cast<double>(i) / (num_steps - 1);
        scan_trajectory.push_back(interpolatePose(start_pose, end_pose, t));
      }

      // Perform the trajectory
      builder.trajectoryMove(scan_trajectory);
      break;
    }

    case CommandKind::CALIBRATE_CAMERA:
    {
      // Example usage:
      //   calibrate_camera <x> <y> <z> <rx> <ry> <rz> <rw>
      // Then we generate 20 different positions (poses) that revolve or offset
      // around this base pose but keep the same "look-at" point.
      if (argc < 11) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: calibrate_camera <x> <y> <z> <rx> <ry> <rz> <rw>");
        rclcpp::shutdown();
        return 1;
      }

      geometry_msgs::msg::Pose center_pose;
      center_pose.position.x    = std::stod(argv[4]);
      center_pose.position.y    = std::stod(argv[5]);
      center_pose.position.z    = std::stod(argv[6]);
      center_pose.orientation.x = std::stod(argv[7]);
      center_pose.orientation.y = std::stod(argv[8]);
      center_pose.orientation.z = std::stod(argv[9]);
      center_pose.orientation.w = std::stod(argv[10]);

      // We'll define 20 poses around this center_pose.
      // For simplicity, let's move in a small circle in XY-plane
      // but keep orientation pointing to the same "look-at" point.
      std::vector<geometry_msgs::msg::Pose> calibration_poses;
      calibration_poses.reserve(20);

      double radius = 0.05; // 5 cm offset in a circle
      for (int i = 0; i < 20; ++i) {
        double angle = (2.0 * M_PI) * (static_cast<double>(i) / 20.0);

        geometry_msgs::msg::Pose p = center_pose;
        // Slightly offset the X/Y around the center
        p.position.x += radius * std::cos(angle);
        p.position.y += radius * std::sin(angle);

        // Orientation logic: we can keep the same orientation
        // or you can recalculate so that the end-effector "points" to center.
        // We'll keep the same orientation for this example.

        calibration_poses.push_back(p);
      }

      builder.trajectoryMove(calibration_poses);
      break;
    }


    case CommandKind::GCODE_LOAD:
    {
      // Example usage:
      //   gcode_load <gcode_filename>
      // We'll parse the G-code lines, convert them to a list of poses,
      // then run trajectoryMove.
      if (argc < 5) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: gcode_load <gcode_filename>");
        rclcpp::shutdown();
        return 1;
      }

      std::string gcode_file = argv[4];
      auto gcode_poses = parseGCodeFile(gcode_file);

      if (gcode_poses.empty()) {
        RCLCPP_ERROR(node->get_logger(),
                     "No valid trajectory from G-code file: %s", gcode_file.c_str());
        rclcpp::shutdown();
        return 1;
      }

      // Save poses to file for debugging
      std::string debug_file = "gcode_poses_debug.txt";
      savePosesToFile(debug_file, gcode_poses);
      RCLCPP_INFO(node->get_logger(), "G-code poses saved to %s", debug_file.c_str());

      builder.trajectoryMove(gcode_poses);
      break;
    }

    case CommandKind::STEP_LOAD:
    {
      // Usage example: step_load <filename>
      if (argc < 5) {
        RCLCPP_ERROR(node->get_logger(), "Usage: step_load <step_filename>");
        rclcpp::shutdown();
        return 1;
      }
      std::string step_filename = argv[4];
      // We'll create a new function to parse the B-spline(s) from the file:
      auto curve_poses = parseStepFile(step_filename);

      // Basic check
      if (curve_poses.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No valid poses generated from STEP file: %s", step_filename.c_str());
        rclcpp::shutdown();
        return 1;
      }

      // Save poses to file for debugging
      // std::string debug_file = "step_curve_poses_debug.txt";
      // savePosesToFile(debug_file, curve_poses);
      // RCLCPP_INFO(node->get_logger(), "STEP curve poses saved to %s", debug_file.c_str());

      // Pass to the normal trajectory move
      builder.trajectoryMove(curve_poses);
      break;
    }


    default:
    {
      RCLCPP_ERROR(node->get_logger(), "Unknown command: %s", command_str.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }


  // Once we've added MTC stages for the chosen command, plan & execute the Task
  if (!builder.initTask()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to init MTC task");
    rclcpp::shutdown();
    return 1;
  }
  if (!builder.planTask()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan MTC task");
    rclcpp::shutdown();
    return 1;
  }
  if (!builder.executeTask() && exec_task) {
    RCLCPP_ERROR(node->get_logger(), "Failed to execute MTC task");
    rclcpp::shutdown();
    return 1;
  }
  if (save_json) {
    RCLCPP_INFO(node->get_logger(), "Saving to JSON file!");
  }

  RCLCPP_INFO(node->get_logger(), "Task complete!");
  rclcpp::shutdown();
  return 0;
}
