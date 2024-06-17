#include "model.h"
#include "geometry.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

Model::Model(const char *filename) : verts_(), faces_() {
  std::ifstream input(filename);
  std::string line;

  while (std::getline(input, line)) {
    std::istringstream in(line);
    std::string prefix;

    in >> prefix;
    if (prefix == "v") {
      Vec3f v;
      for (int i = 0; i < 3; i++) {
        in >> v.raw[i];
      }
      verts_.push_back(v);
    } else if (prefix == "f") {
      std::vector<int> f;
      std::string tmp;
      for (int i = 0; i < 3; i++) {
        in >> tmp;
        std::string raw_face_index = tmp.substr(0, tmp.find("/"));
        f.push_back(std::stoi(raw_face_index) - 1);
      }
      faces_.push_back(f);
    }
  }
}

Model::~Model() {}

int Model::nverts() { return verts_.size(); }

int Model::nfaces() { return faces_.size(); }

Vec3f Model::vert(int idx) { return verts_[idx]; }

std::vector<int> Model::face(int idx) { return faces_[idx]; }
