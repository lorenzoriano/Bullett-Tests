#ifndef __UTILS__H
#define __UTILS__H

#include <vector>
#include <string>
#include <iostream>
#include <sstream>

float string_to_float( const std::string& s )
{
    std::istringstream i(s);
    float x;
    if (!(i >> x)) {
        std::cerr<<"Cannot convert "<<i.str()<<"\n";
        return 0;
    }
    return x;
} 

std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str)
{
    std::vector<std::string>   result;
    std::string                line;
    std::getline(str,line);
    
    std::stringstream          lineStream(line);
    std::string                cell;
    
    while(std::getline(lineStream,cell,' '))
    {
        result.push_back(cell);
    }
    return result;
}

std::vector<std::vector<float> >  parseCvsIntoFloats(std::istream& src) {
    std::string line;
    std::vector<std::vector<float> > values;
    
    while (std::getline(src, line)) {
        if (line[0] == '#')
            continue;
        if (line.size() == 0)
            continue;
        std::istringstream ss(line);
        std::vector< std::string > result = getNextLineAndSplitIntoTokens(ss);
        
        std::vector<float> numbers;
        for (std::vector< std::string >::iterator c = result.begin(); c != result.end(); c++)
            numbers.push_back(string_to_float(*c));
        
        if (numbers.size() == 0)
            continue;
        
        if (!values.empty()) {
            if (numbers.size() != values[0].size()) {
                std::cerr<<"Error reading line: "<<line<<"\n";
                return std::vector< std::vector<float> >();
            }
        }
        values.push_back(numbers);
    }
    return values;
    
}

#endif