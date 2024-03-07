#ifndef TESTBED_IDENTIFIER_H
#define TESTBED_IDENTIFIER_H

#include <vector>
#include <string>
#include <map>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include "Skeleton.h"

using namespace skeleton;
namespace py = pybind11;

namespace identifier {

    class Identifier {
    private:
        int identifier_id;
        std::string identifier_name;
        std::string output_filename;
        std::string ref_filename = "output/me/whole_body.csv";
        std::string openpose_target_filename = "output/openposeTest.csv";
        std::string openpose_ref_filename = "output/2dJoints_v1.4.csv";
        const std::vector<std::string> target_list;
        std::vector<std::map<std::string, std::map<std::string, float>>> result_list;
        bool isWriteToFile = false;
        Skeleton *target_skeleton;

    public:
        Identifier(int id, const std::string &identifier_name, const std::vector<std::string> &target_list,
                   Skeleton *target_skeleton);

        ~Identifier();

        void Identify(int frame);

        /**
         * Write the result to a csv file
         * the Format is:
         * frame, target_name_x, target_name_y, target_name_z, target_name_parent, target_name_child1, target_name_child2, ...
         */
        void WriteOutput();

        void WriteOutput(const std::string &output_filename);

        bool Py_SimilarityScore();

        bool Py_SimilarityScore(const string &output_filename, const string &openpose_target_filename);

        bool Py_SimilarityScore(const string &target_filename, const string &ref_filename, const string &openpose_target_filename,
                                const string &openpose_ref_filename);

        // ------------------------- Getters & Setter ----------------------- //
        const string &GetIdentifierName() const;
    };

    inline const string &Identifier::GetIdentifierName() const {
        return identifier_name;
    }
}

#endif //TESTBED_IDENTIFIER_H
