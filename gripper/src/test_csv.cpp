#include "utils.h"
#include <fstream>

int main(int argc, char** argv) {
    
    if (argc != 2) {
        std::cout<<"Give me the filename!\n";
        return 1;
    }
    
    std::ifstream file(argv[1]);
    
    if (!file.is_open()) {
        std::cerr<<"Error opening file "<<argv[1]<<"\n";
        return 1;
    }
    
    std::vector<std::vector<float> > values;
    values = parseCvsIntoFloats(file);
    
    
    std::cout<<"Values read:\n";
    for (std::vector< std::vector< float > >::const_iterator v = values.begin(); v != values.end(); v++) {
        for (std::vector< float >::const_iterator i = v->begin(); i != v->end(); i++) {
            std::cout<<*i<<" ";
        }
        std::cout<<"\n";
    }
    
    
    return 0;
    
}