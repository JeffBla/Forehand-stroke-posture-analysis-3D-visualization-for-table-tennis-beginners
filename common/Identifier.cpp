#include "Identifier.h"

identifier::Identifier::Identifier(int id, std::string &identifier_name, std::vector<std::string> &target_list,
                                   Skeleton *target_skeleton)
        : identifier_id(id), identifier_name(identifier_name), target_list(target_list),
          target_skeleton(target_skeleton) {
    result_list.resize(target_skeleton->GetBvh()->GetNumFrame());
}

void identifier::Identifier::Identify(int frame) {
    result_list.clear();

    for (const auto &target_name: target_list) {
        auto target_bone = target_skeleton->FindBone(target_name);

        auto angle_info = target_bone->GetAngleInfo();

        result_list[frame][target_name] = angle_info;
    }
}

void identifier::Identifier::WriteOutput() {
    std::ofstream output_file;
    output_file.open("output/" + identifier_name + ".csv");
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
    for (auto &angle_info_list: result_list) {
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
    }
    output_file.close();
}
