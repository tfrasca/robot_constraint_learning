#ifndef WALK_SRC_H
#define WALK_SRC_H

#include "std_msgs/String.h"
namespace walk{

class Listener{
    public:
        Listener();
        ~Listener();
        void callback(const std_msgs::String::ConstPtr& msg);
        int I;
    private:
        int i;
};

}
#endif
