#include "walk_src.h"
//#include "std_msgs/String.h"
namespace walk{

Listener::Listener(){
    this->i = 0;
    this->I = 0;
}

Listener::~Listener(){;}

void Listener::callback(const std_msgs::String::ConstPtr& msg){
    this->I++;
}

}
