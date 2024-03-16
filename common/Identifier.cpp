#include "Identifier.h"

using namespace identifier;

Identifier::Identifier(int id, const std::string &identifier_name, const std::vector<std::string> &target_list,
                       Skeleton *target_skeleton)
        : identifier_id(id), identifier_name(identifier_name), target_list(target_list),
          target_skeleton(target_skeleton) {
    result_list.resize(target_skeleton->GetBvh()->GetNumFrame());
}

Identifier::~Identifier() {
    result_list.clear();
}

void Identifier::Identify(int frame) {
    result_list.resize(target_skeleton->GetBvh()->GetNumFrame());

    for (const auto &target_name: target_list) {
        auto target_bone = target_skeleton->FindBone(target_name);

        auto angle_info = target_bone->GetAngleWithNeighbor();
        auto angle_info_local = target_bone->GetSelfAngle(frame);
        for (const auto &angle: angle_info_local)
            angle_info[angle.first] = angle.second;

        result_list[frame][target_name] = angle_info;
    }
}

void Identifier::WriteOutput(const string &output_filename) {
    std::ofstream output_file;
    this->output_filename = output_filename;
    output_file.open(output_filename);
    if (!output_file.is_open()) {
        cout << "CANNOT OPEN" << endl;
        exit(1);
    }

    /// Write the header
    output_file << "frame,";
    for (const auto &target_name: target_list) {
        output_file << target_name << "_x," << target_name << "_y," << target_name << "_z," << target_name
                    << "_parent,";
        for (const auto &[child_name, child_angle]: result_list[0][target_name]) {
            // Already handled above
            if (child_name.find("self") == std::string::npos && child_name.find("parent") == std::string::npos)
                // Only write the target child
                if (std::find(target_list.begin(), target_list.end(), child_name) != target_list.end())
                    output_file << target_name << "_" << child_name << ",";
        }
    }
    output_file << std::endl;

    /// Write the data
    int frame = 0;
    for (auto &angle_info_list: result_list) {
        output_file << frame << ",";
        for (const auto &target_name: target_list) {
            output_file << angle_info_list[target_name]["self x"] << "," << angle_info_list[target_name]["self y"]
                        << "," << angle_info_list[target_name]["self z"] << ","
                        << angle_info_list[target_name]["parent"] << ",";
            for (const auto &[child_name, child_angle]: angle_info_list[target_name]) {
                // Already handled above
                if (child_name.find("self") == std::string::npos && child_name.find("parent") == std::string::npos)
                    // Only write the target child
                    if (std::find(target_list.begin(), target_list.end(), child_name) != target_list.end())
                        output_file << child_angle << ",";
            }
        }
        output_file << std::endl;
        frame++;
    }
    output_file.close();
}

void Identifier::WriteOutput() {
    output_filename = "output/" + identifier_name + ".csv";
    WriteOutput(output_filename);
}

vector<float>
Identifier::Py_SimilarityScore(const string &target_filename, const string &openpose_target_filename) {
    this->output_filename = target_filename;
    py::module_ PyAnalysizer = py::module_::import("Py_package.PyAnalysizer.Analysize");
    if (identifier_name == "rotation" || identifier_name == "fore_arm") {
        py::object result = PyAnalysizer.attr("ForehandStrokeAnalysis")(openpose_target_filename, target_filename);
        // Output pass or not
        auto prob_result = result.cast<std::vector<float>>();
        return prob_result;
    }

    std::cout << "No such identifier" << std::endl;
    return {};
}

vector<float> Identifier::Py_SimilarityScore() {
    return Py_SimilarityScore(output_filename, openpose_target_filename);
}