#include <iostream>
#include <fstream>
#include <cassert>

int main(){

    std::string conf_file_path = "../../../../config/urdf/kuka_six.urdf";
    std::ifstream ifs(conf_file_path.c_str(), std::ios_base::binary);
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));

    assert(!content.empty());

    return 0;
}