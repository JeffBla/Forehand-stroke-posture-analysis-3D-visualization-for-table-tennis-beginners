#ifndef TESTBED_EVENT_H
#define TESTBED_EVENT_H

#include "Bone.h"

#include <iostream>
#include <functional>
#include <vector>

namespace event{

    template<typename... Args>
    class Event {
    public:
        using EventHandler = std::function<void(Args...)>;

        inline void add_handler(EventHandler handler) {
            handlers.push_back(handler);
        }

        inline void fire(Args... args) {
            for (auto handler: handlers) {
                handler(args...);
            }
        }

    private:
        std::vector<EventHandler> handlers;
    };
}
#endif //TESTBED_EVENT_H
