#ifndef CMU462_MESHEDIT_H
#define CMU462_MESHEDIT_H

#include "halfEdgeMesh.h"
#include <set>
#include <list>
#include <algorithm>
#include <iterator>

using namespace std;

namespace CMU462 {

class MeshResampler {
 public:
  MeshResampler(){};
  ~MeshResampler() {}

  void upsample(HalfedgeMesh& mesh);
  void upsampleSelectedFace(HalfedgeMesh& mesh, list<FaceIter> &faces);
  void downsample(HalfedgeMesh& mesh);
  void resample(HalfedgeMesh& mesh);
};

}  // namespace CMU462

#endif  // CMU462_MESHEDIT_H
