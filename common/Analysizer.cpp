#include "Analysizer.h"

using namespace analysizer;

Analysizer::Analysizer(Skeleton *skeleton) : Analysizer(skeleton, "whole_body") {}

Analysizer::Analysizer(Skeleton *target_skeleton, const std::string &analysizer_name)
        : target_skeleton(target_skeleton), analysizer_name(analysizer_name) {
    output_identifier = new Identifier(0, "whole_body", {target_skeleton->GetTargetBoneNames()}, target_skeleton);

    if (analysizer_name == "forehand_stroke") {
        for (const auto &[identifier_name, target_list]: identifier_target_list.at(analysizer_name)) {
            auto identifier = new Identifier(identifiers.size() + 1, identifier_name, {target_list}, target_skeleton);
            identifiers[identifier_name] = identifier;
        }
    }
}

void Analysizer::_Analyse(map<string, Identifier *> &identifier_list, const string &openposePath) {
    // Analyze the skeleton
    for (int curr_frame = 0; curr_frame < target_skeleton->GetBvh()->GetNumFrame(); curr_frame++) {
        for_each(identifier_list.begin(), identifier_list.end(), [curr_frame](pair<string, Identifier *> element) {
            element.second->Identify(curr_frame);
        });
        target_skeleton->NextBvhMotion();
    }
    // Write the output & analyze
    output_filename = "output/" + analysizer_name + ".csv";
    output_identifier->WriteOutput(output_filename);
    for_each(identifier_list.begin(), identifier_list.end(),
             [&](pair<string, Identifier *> element) {
                 identifier_pass_list[element.first] = element.second->Py_SimilarityScore(output_filename,
                                                                                          openposePath);
             });

    cout << "Analyze done" << endl;
    analysize_done.fire();
}


Analysizer::~Analysizer() {
    identifiers.clear();
}

void Analysizer::Analyze(const string &openposePath) {
    target_skeleton->ApplyBvhMotion(0);
    _Analyse(identifiers, openposePath);
}

void Analysizer::Analyze(Identifier *identifier, const string &openposePath) {
    target_skeleton->ApplyBvhMotion(0);
    map<string, Identifier *> identifier_list = {{identifier->GetIdentifierName(), identifier}};
    _Analyse(identifier_list, openposePath);
}

string Analysizer::Suggest_str(const string &identifier_name) {
    string suggestion;
    if (analysizer_name == "forehand_stroke") {
        /// VTR: 0,1,2: rotation, 3,4,5: fore_arm
        auto prob_vtr = identifier_pass_list[identifier_name];
        if (identifier_name == "rotation") {
            if (prob_vtr[0] >= 0.5) {
                suggestion += "Twist your waist correctly.\n";
            } else {
                if (prob_vtr[1] >= 0.5) {
                    suggestion += "Twist your waist more gently.\n";
                } else if (prob_vtr[2] >= 0.5) {
                    suggestion += "Twist your waist more aggressively.\n";
                } else {
                    suggestion += "Twist your waist totally wrong.\n";
                }
            }
        }
        if (identifier_name == "fore_arm") {
            if (prob_vtr[3] >= 0.5) {
                suggestion += "Wave your arm correctly.\n";
            } else {
                if (prob_vtr[4] >= 0.5) {
                    suggestion += "Wave your arm more gently.\n";
                } else if (prob_vtr[5] >= 0.5) {
                    suggestion += "Wave your arm more aggressively.\n";
                } else {
                    suggestion += "Wave your arm totally wrong.\n";
                }
            }
        }
    }
    // Remove the last '\n'
    suggestion.pop_back();
    return suggestion;
}

string Analysizer::Suggest_str() {
    string suggestion, tmp_suggestion;
    for (const auto &[identifier_name, isPass]: identifier_pass_list) {
        if (identifier_name == "rotation" || identifier_name == "fore_arm") {
            tmp_suggestion = Suggest_str(identifier_name);
        }
        suggestion += tmp_suggestion + "\n";
    }
    // Remove the last '\n'
    suggestion.pop_back();
    return suggestion;
}
