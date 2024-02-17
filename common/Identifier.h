#ifndef TESTBED_IDENTIFIER_H
#define TESTBED_IDENTIFIER_H

#include <vector>
#include <string>
#include <map>

#include "Skeleton.h"

using namespace skeleton;

namespace identifier {

    class Identifier {
    private:
        int identifier_id;
        std::string identifier_name;
        std::vector<std::string> target_list;
        std::vector<std::map<std::string, std::map<std::string, float>>> result_list;
        bool isWriteToFile = false;
        Skeleton *target_skeleton;
    public:
        Identifier(int id, std::string &identifier_name, std::vector<std::string> &target_list,
                   Skeleton *target_skeleton);

        ~Identifier();

        void Identify(int frame);

        /**
         * Write the result to a csv file
         * the Format is:
         * frame, target_name_x, target_name_y, target_name_z, target_name_parent, target_name_child1, target_name_child2, ...
         */
        void WriteOutput();

        // ------------------------- Getters & Setter ----------------------- //
        const string &GetIdentifierName() const;
    };

    inline const string &Identifier::GetIdentifierName() const {
        return identifier_name;
    }
}

#endif //TESTBED_IDENTIFIER_H
