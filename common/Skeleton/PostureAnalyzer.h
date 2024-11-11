#ifndef POSTUREANALYZER_H
#define POSTUREANALYZER_H

#include <vector>
#include "Bone.h"
#include "openglframework.h"

using namespace bone;

namespace skeleton {

    class PostureAnalyzer {
    private:
        std::vector<Bone *> monitoredBones;
        openglframework::Color errorColor;
        openglframework::Color normalColor;

    public:
        void displayValidationResult(const std::string &boneName);
        void clearValidation();
    };

}  // namespace skeleton

#endif