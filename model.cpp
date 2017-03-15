#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.h"

Model::Model(const char *filename) : verts_(), faces_() {
    std::ifstream in;
    std::string line;
    in.open (filename, std::ifstream::in);
    if (in.fail()) return;
    while (!(in.eof())) {
        std::getLine(in,line);
    }
}
