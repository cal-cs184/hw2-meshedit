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
    std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const& points)
    {
        // TODO Part 1.

        std::vector<Vector2D> result;

        for (int i = 0; i < points.size() - 1; i++) {
            Vector2D temp = (1 - t) * points[i] + t * points[i + 1];
            result.push_back(temp);
        }


        return result;
    }

    /**
     * Evaluates one step of the de Casteljau's algorithm using the given points and
     * the scalar parameter t (function parameter).
     *
     * @param points    A vector of points in 3D
     * @param t         Scalar interpolation parameter
     * @return A vector containing intermediate points or the final interpolated vector
     */
    std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const& points, double t) const
    {
        // TODO Part 2.

        std::vector<Vector3D> result;

        for (int i = 0; i < points.size() - 1; i++) {
            Vector3D temp = (1 - t) * points[i] + t * points[i + 1];
            result.push_back(temp);
        }


        return result;

        //return std::vector<Vector3D>();
    }

    /**
     * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
     *
     * @param points    A vector of points in 3D
     * @param t         Scalar interpolation parameter
     * @return Final interpolated vector
     */
    Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const& points, double t) const
    {
        // TODO Part 2.

        Vector3D result;

        if (points.size() == 1) {
            result = points[0];
            return result;
        }

        std::vector<Vector3D> e = evaluateStep(points, t);

        return evaluate1D(e, t);
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
        std::vector<Vector3D> on_u;
        for (int i = 0; i < controlPoints.size(); i++) {
            on_u.push_back(evaluate1D(controlPoints[i], u));
        }

        Vector3D on_v;
        on_v = evaluate1D(on_u, v);


        return on_v;
    }

    Vector3D Vertex::normal(void) const
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

        return normal / normal.norm() + 1e-10;

    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0)
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

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0)
    {
        // TODO Part 5.
        // This method should split the given edge and return an iterator to the newly inserted vertex.
        // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

        if (e0->isBoundary()) {

            // This part focuses on the handle of boundary(the part of extra points)
            // Our notes on new picture before and after boundary splitting can be seen on https://lzztypora.oss-cn-beijing.aliyuncs.com/202402261559104.png

            // Step1: Draw every elements just like before
            /*
            FaceIter f111 = e0->halfedge()->face();
            HalfedgeIter h0 = e0->halfedge()->twin();
            for (FaceCIter b = facesBegin(); b != facesEnd(); b++) {
                if(b == f111){ HalfedgeIter h0 = e0->halfedge(); }
            }
            */

            HalfedgeIter h0 = e0->halfedge();
            HalfedgeIter h1 = h0->next();
            HalfedgeIter h2 = h1->next();
            HalfedgeIter h3 = h0->twin();
            HalfedgeIter h4 = h2->twin();
            HalfedgeIter h5 = h1->twin();

            VertexIter v0 = h3->vertex();
            VertexIter v1 = h4->vertex();
            VertexIter v2 = h5->vertex();

            /*v0->isNew = false;
            v1->isNew = false; 
            v2->isNew = false;*/

            EdgeIter e1 = h1->edge();
            EdgeIter e2 = h2->edge();

            /*e1->isNew = false;
            e2->isNew = false;*/

            FaceIter f0 = h0->face();

            // Step2: Allocate new elements
            // 4 half-edge, 1 vertex, 2 edges, 1 faces
            HalfedgeIter h6 = newHalfedge();
            HalfedgeIter h7 = newHalfedge();
            HalfedgeIter h8 = newHalfedge();
            HalfedgeIter h9 = newHalfedge();

            VertexIter v3 = newVertex();
            v3->position = (v0->position + v1->position) / 2;

            v3->isNew = true;

            EdgeIter e3 = newEdge();
            EdgeIter e4 = newEdge();

            //e3->isNew = true;
            //e4->isNew = true;

            FaceIter f1 = newFace();

            // Step3: Reassign elements
            /*
            void setNeighbors( HalfedgeIter next,
                            HalfedgeIter twin,
                            VertexIter vertex,
                            EdgeIter edge,
                            FaceIter face )
            */
            h0->setNeighbors(h6, h3, v1, e0, f0);
            h1->setNeighbors(h7, h5, v0, e1, f1);
            h2->setNeighbors(h0, h4, v2, e2, f0);
            h3->setNeighbors(h3->next(), h0, v3, e0, h3->face());
            h4->setNeighbors(h4->next(), h2, v1, e2, f0);
            h5->setNeighbors(h5->next(), h1, v2, e1, f1);
            h6->setNeighbors(h2, h7, v3, e4, f0);
            h7->setNeighbors(h8, h6, v2, e4, f1);
            h8->setNeighbors(h1, h9, v3, e3, f1);
            h9->setNeighbors(h3, h8, v0, e3, h3->face());

            v0->halfedge() = h4;
            v1->halfedge() = h1;
            v2->halfedge() = h7;
            v3->halfedge() = h6;

            e0->halfedge() = h0;
            e1->halfedge() = h4;
            e2->halfedge() = h1;
            e3->halfedge() = h8;
            e4->halfedge() = h6;

            f0->halfedge() = h0;
            f1->halfedge() = h8;

            cout << "Boundary" << endl;
            return v3;
        }


        else {
            // This part is for the normal condition without boundary.
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

            /*e1->isNew = false;
            e2->isNew = false;
            e3->isNew = false;
            e4->isNew = false;*/

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
            v4->position = (v0->position + v1->position) / 2;

            v4->isNew = true;

            EdgeIter e5 = newEdge();
            EdgeIter e6 = newEdge();
            EdgeIter e7 = newEdge();

            FaceIter f2 = newFace();
            FaceIter f3 = newFace();

            e5->isNew = true;
            e6->isNew = true;
            //e5->isNew = false;
            //e6->isNew = false;
            e7->isNew = true;

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

            //set some e to false???
            e5->isNew = true;
            e6->isNew = true;
            e7->isNew = true;

            f0->halfedge() = h0;
            f1->halfedge() = h3;
            f2->halfedge() = h11;
            f3->halfedge() = h10;

            return v4;
        }
    }



    void MeshResampler::upsample(HalfedgeMesh& mesh)
    {
        // TODO Part 6.
        // This routine should increase the number of triangles in the mesh using Loop subdivision.
        // One possible solution is to break up the method as listed below.

          // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
          // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
          // a vertex of the original mesh.
        
        // set up mesh
       // int half_edge_counter = 0;

         // loop through each vertex in mesh
         // while the curr half_edge != intial half_edge of whole mesh
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            
          // Define:
          // - sum of neighbors' positions
          // - curr half_edge for looping through neighbor
          // - vertex of current half_edge
          // - counter for degree of current vertex

            Vector3D neighbors_pos = Vector3D(0, 0, 0);
            //changed to half_edge
            HalfedgeIter neighbor_iter = v->halfedge();
            int vertexDegree = v->degree();
            float vertex_const= 0.0;

            // set all old verticies = not new
            v->isNew = false;

            // loop through each edge for this specific vertex
            // while the curr half_edge != intial half_edge of this group of vertices
            do {
                // Sum neighbors position, add 1 vertex degree
                neighbors_pos += neighbor_iter->vertex()->position;
                //vertexDegree += 1;

                // move to next halfedge of vertex
                neighbor_iter = neighbor_iter->twin()->next();

            } while (neighbor_iter != v->halfedge());

            // check vertexDegree
            //cout << vertexDegree;
            if (vertexDegree == 3) {
                vertex_const = (float) 3 / 16;
            }
            else {
                vertex_const = (float) 3 / (8 * vertexDegree);
            }
            //cout << vertex_const;

            // vertex->degree??
            // set new position for curr vertex
            v->newPosition = ((1 - vertexDegree * vertex_const) * v->position) + (vertex_const * neighbors_pos);
           // cout << v->newPosition;
        }
        // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

          //  loop through each edge
       // mesh.flipEdge(mesh.edgesBegin());
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            // create inital values
            // EdgeIter curr_edge = e;
            //e->isNew = false

            //might be off AHHHHHHHHHHHHHH
            // find surrounding vertices of rhombus (2 triangles)
            VertexIter v_a = e->halfedge()->vertex();
            VertexIter v_b = e->halfedge()->next()->vertex();
            VertexIter v_c = e->halfedge()->next()->next()->vertex();
            // this is for the vertex not in the triangle
            VertexIter v_d = e->halfedge()->twin()->next()->next()->vertex();

            // figure out position for surrounding vertices
            Vector3D v_a_pos = v_a->position;
            Vector3D v_b_pos = v_b->position;
            Vector3D v_c_pos = v_c->position;
            Vector3D v_d_pos = v_d->position;

            //calculate newedge position
            e->newPosition = (((float) 3 / 8) * (v_a_pos + v_b_pos)) + (((float) 1 / 8) * (v_c_pos + v_d_pos));
            //cout <<  (((float) 3 / 8) * (v_a_pos + v_b_pos)) + (((float) 1 / 8) * (v_c_pos + v_d_pos));
        }

        // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
// information about which subdivide edges come from splitting an edge in the original mesh, and which edges
// are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
// the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)


  // loop through each edge for this specific vertex
  // while the curr half_edge != intial half_edge of this group of vertices

        vector<EdgeIter> originalEdges;
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            originalEdges.push_back(e);
        }

        //cout << mesh.nHalfedges();
        for (auto e = originalEdges.begin(); e != originalEdges.end(); e++) {

            // VertexIter curr_vertex = (*e)->halfedge()->vertex();
            // VertexIter connected_vertex = e->halfedge()->twin()->vertex();

            // Start time
            auto start = std::chrono::high_resolution_clock::now();

            VertexIter new_vertex = mesh.splitEdge(*e);
            new_vertex->isNew = true;
            new_vertex->newPosition = (*e)->newPosition;
            new_vertex->halfedge()->edge()->isNew = false;
            new_vertex->halfedge()->twin()->next()->edge()->isNew = true;
            new_vertex->halfedge()->next()->next()->edge()->isNew = true;
            new_vertex->halfedge()->twin()->next()->twin()->next()->edge()->isNew = false;


            auto stop = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);


            // std::cout << "Time taken by iteration: " << duration.count() << " microseconds" << std::endl;


        }

        // 4. Flip any new edge that connects an old and new vertex.
        // 5. Copy the new vertex positions into final Vertex::position.

        // initilize curr half_edge and half_edge counter
            //mesh.flipEdge(mesh.edgesBegin());
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            VertexIter curr_vertex = e->halfedge()->vertex();
            VertexIter connected_vertex = e->halfedge()->twin()->vertex();

            if (e->isNew == true) {
                // if curr vertex is old and connecting vertex new then flip curr_edge
                // || curr_vertex->isNew && !connected_vertex->isNew
                // check one new! isEqual to other new
                if (curr_vertex->isNew != connected_vertex->isNew) {
                    mesh.flipEdge(e);
                }
               // curr_vertex->position = e->newPosition;
            }
           /* else {
                curr_vertex->position = curr_vertex->newPosition;
            }*/
        }
        
        for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++)
        {
            
            if (v->isNew) {
                v->position = v->halfedge()->next()->edge()->newPosition;
                cout << v->halfedge()->next()->edge()->newPosition;
            }
            else{
                v->position = v->newPosition;
            }
            //cout << v->position;
        }
        
                // set position final for inner edges (non-boundary)
                //curr_vertex->position = e->newPosition;

            //}
          
            // set position final for boundary edges
            
       // }
  }

}
