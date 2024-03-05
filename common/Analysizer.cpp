#include "Analysizer.h"

using namespace analysizer;

Analysizer::Analysizer(Skeleton *skeleton, SceneDemo *scene) : Analysizer(skeleton,
                                                                                      {"whole_body", "foreArm_hand",
                                                                                       "shin_foot", "body_rotate"},
                                                                                      scene) {}

Analysizer::Analysizer(Skeleton *target_skeleton, std::vector<std::string> identifier_name_list,
                                   SceneDemo *scene)
        : target_skeleton(target_skeleton), identifier_name_list(identifier_name_list), scene(scene) {
    for (int id = 0; id < identifier_name_list.size(); id++) {
        auto identifier_name = identifier_name_list[id];
        std::vector<std::string> target_list;
        // Set up the targets of identifier
        if (identifier_name == "whole_body") {
            target_list = target_skeleton->GetTargetBoneNames();
        } else if (identifier_name == "foreArm_hand") {
            target_list = {"lForeArm", "lHand", "rForeArm", "rHand"};
        } else if (identifier_name == "shin_foot") {
            target_list = {"lShin", "lThigh", "rShin", "rThigh"};
        } else if (identifier_name == "body_rotate") {
            target_list = {"hip"};
        } else {
            std::cout << "Error: identifier name not found" << std::endl;
            exit(1);
        }

        auto *identifier = new Identifier(id, identifier_name, {target_list}, target_skeleton);
        identifiers[identifier_name] = identifier;
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
    // Write the output
    for_each(identifier_list.begin(), identifier_list.end(), [openposePath](pair<string, Identifier *> element) {
        element.second->WriteOutput();

        element.second->Py_SimilarityScore(openposePath);
    });

    cout << "Analyze done" << endl;
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
