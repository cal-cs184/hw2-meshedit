#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    
    return std::vector<Vector2D>();
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    return std::vector<Vector3D>();
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    return Vector3D();
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    return Vector3D();
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    // cout << "Where you are";

    Vector3D normal(0, 0, 0);

    HalfedgeCIter h_origin = halfedge(); //用上一层级
    VertexCIter v_origin = h_origin->vertex();

    // Use class Edge
    do {
        // Calculate the two other points in the same triangle
        VertexCIter v1 = h_origin->twin()->vertex(); 
        VertexCIter v2 = h_origin->twin()->next()->twin()->vertex(); 
        Vector3D linetwin = v1->position - v_origin->position;
        Vector3D linenext = v2->position - v_origin->position;
        Vector3D normal_eachface = cross(-linetwin, linenext); //Notice the direction of normal
        
        float area = 0.5 * normal_eachface.norm();
        normal += area * normal_eachface;

        h_origin = h_origin->twin()->next();   

    } while (h_origin != v_origin->halfedge());          
    //cout << normal / normal.norm()<<endl;
    
    return normal/normal.norm()+1e-10;
    
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    // Refer from the class materials:http://15462.courses.cs.cmu.edu/fall2015content/misc/HalfedgeEdgeOpImplementationGuide.pdf
    
    if (e0->isBoundary()) { return e0; }

    // Step1: Draw every elements
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h6->vertex();
    VertexIter v3 = h8->vertex();

    // EdgeIter e0 = h0->edge();
    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();

    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    //Step2: Reassign elements
    //setNeighbors(...)
    h0->next() = h1;
    h0->twin() = h3;
    h0->vertex() = v3;
    h0->edge() = e0;
    h0->face() = f0; //All faces are not changed
    // A debug there is that we should use EdgeIter other EdgeCIter
    
    h1->next() = h2;
    h1->twin() = h7;
    h1->vertex() = v2;
    h1->edge() = e2;
    h1->face() = f0;

    h2->next() = h0;
    h2->twin() = h8;
    h2->vertex() = v0;
    h2->edge() = e3;
    h2->face() = f0;

    h3->next() = h4;
    h3->twin() = h0;
    h3->vertex() = v2;
    h3->edge() = e0;
    h3->face() = f1;

    h4->next() = h5;
    h4->twin() = h9;
    h4->vertex() = v3;
    h4->edge() = e4;
    h4->face() = f1;

    h5->next() = h3;
    h5->twin() = h6;
    h5->vertex() = v1;
    h5->edge() = e1;
    h5->face() = f1;

    h6->next() = h6->next(); // Stays the same, but set it anyway
    h6->twin() = h5;
    h6->vertex() = v2;
    h6->edge() = e1;
    h6->face() = h6->face(); // Stays the same, but set it anyway

    h7->next() = h7->next(); // Stays the same, but set it anyway
    h7->twin() = h1;
    h7->vertex() = v0;
    h7->edge() = e2; 
    h7->face() = h7->face(); // Stays the same, but set it anyway

    h8->next() = h8->next(); // Stays the same, but set it anyway
    h8->twin() = h2;
    h8->vertex() = v3;
    h8->edge() = e3; 
    h8->face() = h8->face(); // Stays the same, but set it anyway

    h9->next() = h9->next(); // Stays the same, but set it anyway
    h9->twin() = h4;
    h9->vertex() = v1; 
    h9->edge() = e4; 
    h9->face() = h9->face(); // Stays the same, but set it anyway

    v0->halfedge() = h2;
    v1->halfedge() = h5;
    v2->halfedge() = h3;
    v3->halfedge() = h0; //Maybe if we hava many choice, we can randomly change one of them.

    e0->halfedge() = h0;
    e1->halfedge() = h5; 
    e2->halfedge() = h1;
    e3->halfedge() = h2;
    e4->halfedge() = h4;

    f0->halfedge() = h0;
    f1->halfedge() = h3;

    // Return the flipped edge
    return e0;


  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    
    // Our notes on new picture after splitting can be seen on https://lzztypora.oss-cn-beijing.aliyuncs.com/202402251921116.png.
    // Step1: Draw every elements just like before
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h6->vertex();
    VertexIter v3 = h8->vertex();

    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();

    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();
    
    // Step2: Allocate new elements
    // 6 half-edge, 1 vertex, 3 edges, 2 faces
    HalfedgeIter h10 = newHalfedge();
    HalfedgeIter h11 = newHalfedge();
    HalfedgeIter h12 = newHalfedge();
    HalfedgeIter h13 = newHalfedge();
    HalfedgeIter h14 = newHalfedge();
    HalfedgeIter h15 = newHalfedge();

    VertexIter v4 = newVertex();
    v4->position = (v0->position + v1-> position)/2;

    EdgeIter e5 = newEdge();
    EdgeIter e6 = newEdge();
    EdgeIter e7 = newEdge();

    FaceIter f2 = newFace();
    FaceIter f3 = newFace();

    // Step3: Reassign elements
    /*
    void setNeighbors( HalfedgeIter next,
                    HalfedgeIter twin,
                    VertexIter vertex,
                    EdgeIter edge,
                    FaceIter face )
    */
    h0->setNeighbors(h1, h3, v4, e0, f0);
    h1->setNeighbors(h13, h6, v1, e1, f0);
    h2->setNeighbors(h11, h7, v2, e2, f2);
    h3->setNeighbors(h12, h0, v1, e0, f1);
    h4->setNeighbors(h14, h8, v0, e3, f3);
    h5->setNeighbors(h3, h9, v3, e4, f1);
    h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
    h7->setNeighbors(h7->next(), h2, v0, e2, h7->face());
    h8->setNeighbors(h8->next(), h4, v3, e3, h8->face());
    h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());
    h10->setNeighbors(h4, h11, v4, e7, f3);
    h11->setNeighbors(h15, h10, v0, e7, f2);
    h12->setNeighbors(h5, h14, v4, e5, f1);
    h13->setNeighbors(h0, h15, v2, e6, f0);
    h14->setNeighbors(h10, h12, v3, e5, f3);
    h15->setNeighbors(h2, h13, v4, e6, f2);

    v0->halfedge() = h7;
    v1->halfedge() = h9;
    v2->halfedge() = h6;
    v3->halfedge() = h8;
    v4->halfedge() = h0;

    e0->halfedge() = h0;
    e1->halfedge() = h1;
    e2->halfedge() = h2;
    e3->halfedge() = h4;
    e4->halfedge() = h9;
    e5->halfedge() = h12;
    e6->halfedge() = h15;
    e7->halfedge() = h10;

    f0->halfedge() = h0;
    f1->halfedge() = h3;
    f2->halfedge() = h11;
    f3->halfedge() = h10;

    return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

  }
}
