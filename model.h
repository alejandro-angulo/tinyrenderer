// I copied this from ssloy's repo

#ifndef __MODEL_H__
#define __MODEL_H__

#include "geometry.h"
#include <vector>

class Model {
private:
  std::vector<Vec3f> verts_;

  // Holds corresponding indices in verts_
  std::vector<std::vector<int>> faces_;

public:
  Model(const char *);
  ~Model();
  int nverts();
  int nfaces();
  Vec3f vert(int);
  std::vector<int> face(int);
};

#endif // __MODEL_H__
