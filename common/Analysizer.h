#ifndef TESTBED_ANALYSIZER_H
#define TESTBED_ANALYSIZER_H

#include <map>

#include "Identifier.h"
#include "Skeleton.h"
#include "SceneDemo.h"

using namespace identifier;
using namespace skeleton;

namespace analysizer {
    class Analysizer {
    private:
        // ------------------------- Attributes ----------------------- //
        SceneDemo *scene;

        Skeleton *target_skeleton;

        std::map<std::string, Identifier *> identifiers;

        std::vector<std::string> identifier_name_list;

        // ------------------------- Methods ----------------------- //
        void _Analyse(map<string, Identifier *> &identifier_list, const string &openposePath);

    public:
        Analysizer(Skeleton *skeleton, SceneDemo *scene);

        Analysizer(Skeleton *target_skeleton, std::vector<std::string> identifier_name_list,
                   SceneDemo *scene);

        ~Analysizer();

        /**
         * Analyze the skeleton with all the identifiers
         */
        void Analyze(const string &openposePath);

        /**
         * Analyze the skeleton with the given identifier
         * @param identifier
         */
        void Analyze(Identifier *identifier, const string &openposePath);

        void WriteOutput(int identitier_id);

        void WriteOutput(std::string identifier_name);

        void WriteOutput();
    };

}


#endif //TESTBED_ANALYSIZER_H
