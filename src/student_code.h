#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

#include "halfEdgeMesh.h"
#include "bezierPatch.h"
#include "bezierCurve.h"

using namespace std; 
// 这意味着代码中的所有 std 命名空间下的符号都可以直接使用，
// 而不需要在使用它们时加上 std:: 前缀

namespace CGL {

  class MeshResampler{

  public:

    MeshResampler(){};
    ~MeshResampler(){}

    void upsample(HalfedgeMesh& mesh);
  };
}

#endif // STUDENT_CODE_H
