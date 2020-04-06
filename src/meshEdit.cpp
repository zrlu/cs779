#include <float.h>
#include <assert.h>
#include "meshEdit.h"
#include "mutablePriorityQueue.h"
#include "error_dialog.h"


namespace CMU462 {

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should split the given edge and return an iterator to the
  // newly inserted vertex. The halfedge of this vertex should point along
  // the edge that was split, rather than the new edges.

  list<VertexIter> newVertices;
  list<EdgeIter> newEdges;
  list<FaceIter> newFaces;

  return splitEdge(e0, newVertices, newEdges, newFaces);
}

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0, list<VertexIter>& newVertices, list<EdgeIter>& newEdges, list<FaceIter>& newFaces)
{
  FaceIter f0 = e0->halfedge()->face();
  FaceIter f1 = e0->halfedge()->twin()->face();

  if (f1->isBoundary()) {
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h1->twin();
    HalfedgeIter h5 = h2->twin();
    HalfedgeIter h6 = newHalfedge();
    HalfedgeIter h7 = newHalfedge();
    HalfedgeIter h8 = newHalfedge();
    HalfedgeIter h9 = newHalfedge();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h1->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = newVertex();
    v3->position = (v0->position + v1->position) / 2;
    newVertices.push_back(v3);

    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = newEdge();
    EdgeIter e4 = newEdge();
    newEdges.push_back(e3);
    newEdges.push_back(e4);

    FaceIter f2 = newFace();
    newFaces.push_back(f2);

    if (!f0->getFace()->alreadySplitted) {
      f0->getFace()->subdivisionLevel += 1;
      f0->getFace()->alreadySplitted = true;
    }

    f2->getFace()->subdivisionLevel = f0->getFace()->subdivisionLevel;
    f2->getFace()->alreadySplitted = true;

    h0->setNeighbors(h9, h7, v0, e0, f0);
    h1->setNeighbors(h8, h4, v1, e1, f2);
    h2->setNeighbors(h0, h5, v2, e2, f0);
    h7->setNeighbors(h3->next(), h0, v3, e0, f1);
    h3->setNeighbors(h7, h6, v1, e3, f1);
    h6->setNeighbors(h1, h3, v3, e3, f2);
    h8->setNeighbors(h6, h9, v2, e4, f2);
    h9->setNeighbors(h2, h8, v3, e4, f0);

    e0->halfedge() = h0;
    e1->halfedge() = h1;
    e2->halfedge() = h2;
    e3->halfedge() = h6;
    e4->halfedge() = h9;

    f0->halfedge() = h0;
    f1->halfedge() = h3;
    f2->halfedge() = h6;

    v0->halfedge() = v0->halfedge();
    v1->halfedge() = h3;
    v2->halfedge() = h4;
    v3->halfedge() = h7;

    return v3;
  }

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
  HalfedgeIter h10 = newHalfedge();
  HalfedgeIter h11 = newHalfedge();
  HalfedgeIter h12 = newHalfedge();
  HalfedgeIter h13 = newHalfedge();
  HalfedgeIter h14 = newHalfedge();
  HalfedgeIter h15 = newHalfedge();

  VertexIter v0 = h0->vertex();
  VertexIter v1 = h3->vertex();
  VertexIter v2 = h6->vertex();
  VertexIter v3 = h8->vertex();

  EdgeIter e1 = h1->edge();
  EdgeIter e2 = h2->edge();
  EdgeIter e3 = h4->edge();
  EdgeIter e4 = h5->edge();
  EdgeIter e5 = newEdge();
  EdgeIter e6 = newEdge();
  EdgeIter e7 = newEdge();
  newEdges.push_back(e5);
  newEdges.push_back(e6);
  newEdges.push_back(e7);

  FaceIter f2 = newFace();
  FaceIter f3 = newFace();
  newFaces.push_back(f2);
  newFaces.push_back(f3);

  if (!f0->getFace()->alreadySplitted) {
    f0->getFace()->subdivisionLevel += 1;
    f0->getFace()->alreadySplitted = true;
  }
  if (!f1->getFace()->alreadySplitted) {
    f1->getFace()->subdivisionLevel += 1;
    f1->getFace()->alreadySplitted = true;
  }

  f2->getFace()->subdivisionLevel = f0->getFace()->subdivisionLevel;
  f2->getFace()->alreadySplitted = true;

  f3->getFace()->subdivisionLevel = f1->getFace()->subdivisionLevel;
  f3->getFace()->alreadySplitted = true;


  VertexIter v4 = newVertex();
  v4->position = (v0->position + v1->position) / 2;
  newVertices.push_back(v4);


  h0->setNeighbors(h1, h3, v4, e0, f0);
  h1->setNeighbors(h10, h6, v1, e1, f0);
  h2->setNeighbors(h14, h7, v2, e2, f3);
  h3->setNeighbors(h11, h0, v1, e0, f1);
  h4->setNeighbors(h12, h8, v0, e3, f2);
  h5->setNeighbors(h3, h9, v3, e4, f1);
  h6->setNeighbors(h6->next(), h1, v2, e1, h6->face());
  h7->setNeighbors(h7->next(), h2, v0, e2, h7->face());
  h8->setNeighbors(h8->next(), h4, v3, e3, h8->face());
  h9->setNeighbors(h9->next(), h5, v1, e4, h9->face());
  h10->setNeighbors(h0, h15, v2, e5, f0);
  h11->setNeighbors(h5, h12, v4, e7, f1);
  h12->setNeighbors(h13, h11, v3, e7, f2);
  h13->setNeighbors(h4, h14, v4, e6, f2);
  h14->setNeighbors(h15, h13, v0, e6, f3);
  h15->setNeighbors(h2, h10, v4, e5, f3);

  e0->halfedge() = h0;
  e1->halfedge() = h1;
  e2->halfedge() = h2;
  e3->halfedge() = h4;
  e4->halfedge() = h5;
  e5->halfedge() = h10;
  e6->halfedge() = h14;
  e7->halfedge() = h11;

  f0->halfedge() = h0;
  f1->halfedge() = h3;
  f2->halfedge() = h4;
  f3->halfedge() = h2;

  v0->halfedge() = h4;
  v1->halfedge() = h3;
  v2->halfedge() = h2;
  v3->halfedge() = h5;
  v4->halfedge() = h0;

  return v4;
}

VertexIter HalfedgeMesh::collapseEdge(EdgeIter e) {
  // TODO: (meshEdit)
  // This method should collapse the given edge and return an iterator to
  // the new vertex created by the collapse.

  showError("collapseEdge() not implemented.");
  return VertexIter();
}

VertexIter HalfedgeMesh::collapseFace(FaceIter f) {
  // TODO: (meshEdit)
  // This method should collapse the given face and return an iterator to
  // the new vertex created by the collapse.
  showError("collapseFace() not implemented.");
  return VertexIter();
}

FaceIter HalfedgeMesh::eraseVertex(VertexIter v) {
  // TODO: (meshEdit)
  // This method should replace the given vertex and all its neighboring
  // edges and faces with a single face, returning the new face.

  return FaceIter();
}

FaceIter HalfedgeMesh::eraseEdge(EdgeIter e) {
  // TODO: (meshEdit)
  // This method should erase the given edge and return an iterator to the
  // merged face.

  showError("eraseVertex() not implemented.");
  return FaceIter();
}

EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should flip the given edge and return an iterator to the
  // flipped edge.

  FaceIter f0 = e0->halfedge()->face();
  FaceIter f1 = e0->halfedge()->twin()->face();

  if (f1->isBoundary()) {
    return e0;
  }
  if (!e0->flipped) {
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

    h0->setNeighbors(h1, h3, v3, e0, f0);
    h1->setNeighbors(h2, h7, v2, e2, f0);
    h2->setNeighbors(h0, h8, v0, e3, f0);

    h3->setNeighbors(h4, h0, v2, e0, f1);
    h4->setNeighbors(h5, h9, v3, e4, f1);
    h5->setNeighbors(h3, h6, v1, e1, f1);

    h6->twin() = h5;
    h7->twin() = h1;
    h8->twin() = h2;
    h9->twin() = h4;

    v0->halfedge() = h7;
    v1->halfedge() = h9;
    v2->halfedge() = h6;
    v3->halfedge() = h8;

    e0->halfedge() = h0;
    e1->halfedge() = h5;
    e2->halfedge() = h1;
    e3->halfedge() = h2;
    e4->halfedge() = h4;

    f0->halfedge() = h0;
    f1->halfedge() = h3;

    e0->flipped = true;
  } else {
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h5->twin();
    HalfedgeIter h7 = h1->twin();
    HalfedgeIter h8 = h2->twin();
    HalfedgeIter h9 = h4->twin();

    VertexIter v0 = h7->vertex();
    VertexIter v1 = h9->vertex();
    VertexIter v2 = h6->vertex();
    VertexIter v3 = h8->vertex();

    EdgeIter e1 = h5->edge();
    EdgeIter e2 = h1->edge();
    EdgeIter e3 = h2->edge();
    EdgeIter e4 = h4->edge();

    h0->setNeighbors(h1, h3, v0, e0, f0);
    h1->setNeighbors(h2, h6, v1, e1, f0);
    h2->setNeighbors(h0, h7, v2, e2, f0);

    h3->setNeighbors(h4, h0, v1, e0, f1);
    h4->setNeighbors(h5, h8, v0, e3, f1);
    h5->setNeighbors(h3, h9, v3, e4, f1);

    h6->twin() = h1;
    h7->twin() = h2;
    h8->twin() = h4;
    h9->twin() = h5;

    v0->halfedge() = h7;
    v1->halfedge() = h9;
    v2->halfedge() = h6;
    v3->halfedge() = h8;

    e0->halfedge() = h0;
    e1->halfedge() = h6;
    e2->halfedge() = h7;
    e3->halfedge() = h8;
    e4->halfedge() = h9;

    f0->halfedge() = h0;
    f1->halfedge() = h3;

    e0->flipped = false;
  }

  return e0;
}

void HalfedgeMesh::subdivideQuad(bool useCatmullClark) {
  // Unlike the local mesh operations (like bevel or edge flip), we will perform
  // subdivision by splitting *all* faces into quads "simultaneously."  Rather
  // than operating directly on the halfedge data structure (which as you've
  // seen
  // is quite difficult to maintain!) we are going to do something a bit nicer:
  //
  //    1. Create a raw list of vertex positions and faces (rather than a full-
  //       blown halfedge mesh).
  //
  //    2. Build a new halfedge mesh from these lists, replacing the old one.
  //
  // Sometimes rebuilding a data structure from scratch is simpler (and even
  // more
  // efficient) than incrementally modifying the existing one.  These steps are
  // detailed below.

  // TODO Step I: Compute the vertex positions for the subdivided mesh.  Here
  // we're
  // going to do something a little bit strange: since we will have one vertex
  // in
  // the subdivided mesh for each vertex, edge, and face in the original mesh,
  // we
  // can nicely store the new vertex *positions* as attributes on vertices,
  // edges,
  // and faces of the original mesh.  These positions can then be conveniently
  // copied into the new, subdivided mesh.
  // [See subroutines for actual "TODO"s]
  if (useCatmullClark) {
    computeCatmullClarkPositions();
  } else {
    computeLinearSubdivisionPositions();
  }

  // TODO Step II: Assign a unique index (starting at 0) to each vertex, edge,
  // and
  // face in the original mesh.  These indices will be the indices of the
  // vertices
  // in the new (subdivided mesh).  They do not have to be assigned in any
  // particular
  // order, so long as no index is shared by more than one mesh element, and the
  // total number of indices is equal to V+E+F, i.e., the total number of
  // vertices
  // plus edges plus faces in the original mesh.  Basically we just need a
  // one-to-one
  // mapping between original mesh elements and subdivided mesh vertices.
  // [See subroutine for actual "TODO"s]
  assignSubdivisionIndices();

  // TODO Step III: Build a list of quads in the new (subdivided) mesh, as
  // tuples of
  // the element indices defined above.  In other words, each new quad should be
  // of
  // the form (i,j,k,l), where i,j,k and l are four of the indices stored on our
  // original mesh elements.  Note that it is essential to get the orientation
  // right
  // here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces should
  // circulate in the same direction as old faces (think about the right-hand
  // rule).
  // [See subroutines for actual "TODO"s]
  vector<vector<Index> > subDFaces;
  vector<Vector3D> subDVertices;
  buildSubdivisionFaceList(subDFaces);
  buildSubdivisionVertexList(subDVertices);

  // TODO Step IV: Pass the list of vertices and quads to a routine that clears
  // the
  // internal data for this halfedge mesh, and builds new halfedge data from
  // scratch,
  // using the two lists.
  rebuild(subDFaces, subDVertices);
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * simple linear interpolation, e.g., the edge midpoints and face
 * centroids.
 */
void HalfedgeMesh::computeLinearSubdivisionPositions() {
  // TODO For each vertex, assign Vertex::newPosition to
  // its original position, Vertex::position.

  // TODO For each edge, assign the midpoint of the two original
  // positions to Edge::newPosition.

  // TODO For each face, assign the centroid (i.e., arithmetic mean)
  // of the original vertex positions to Face::newPosition.  Note
  // that in general, NOT all faces will be triangles!
  showError("computeLinearSubdivisionPositions() not implemented.");
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * the Catmull-Clark rules for subdivision.
 */
void HalfedgeMesh::computeCatmullClarkPositions() {
  // TODO The implementation for this routine should be
  // a lot like HalfedgeMesh::computeLinearSubdivisionPositions(),
  // except that the calculation of the positions themsevles is
  // slightly more involved, using the Catmull-Clark subdivision
  // rules. (These rules are outlined in the Developer Manual.)

  // TODO face

  // TODO edges

  // TODO vertices
  showError("computeCatmullClarkPositions() not implemented.");
}

/**
 * Assign a unique integer index to each vertex, edge, and face in
 * the mesh, starting at 0 and incrementing by 1 for each element.
 * These indices will be used as the vertex indices for a mesh
 * subdivided using Catmull-Clark (or linear) subdivision.
 */
void HalfedgeMesh::assignSubdivisionIndices() {
  // TODO Start a counter at zero; if you like, you can use the
  // "Index" type (defined in halfedgeMesh.h)

  // TODO Iterate over vertices, assigning values to Vertex::index

  // TODO Iterate over edges, assigning values to Edge::index

  // TODO Iterate over faces, assigning values to Face::index
  showError("assignSubdivisionIndices() not implemented.");
}

/**
 * Build a flat list containing all the vertex positions for a
 * Catmull-Clark (or linear) subdivison of this mesh.  The order of
 * vertex positions in this list must be identical to the order
 * of indices assigned to Vertex::newPosition, Edge::newPosition,
 * and Face::newPosition.
 */
void HalfedgeMesh::buildSubdivisionVertexList(vector<Vector3D>& subDVertices) {
  // TODO Resize the vertex list so that it can hold all the vertices.

  // TODO Iterate over vertices, assigning Vertex::newPosition to the
  // appropriate location in the new vertex list.

  // TODO Iterate over edges, assigning Edge::newPosition to the appropriate
  // location in the new vertex list.

  // TODO Iterate over faces, assigning Face::newPosition to the appropriate
  // location in the new vertex list.
  showError("buildSubdivisionVertexList() not implemented.");
}

/**
 * Build a flat list containing all the quads in a Catmull-Clark
 * (or linear) subdivision of this mesh.  Each quad is specified
 * by a vector of four indices (i,j,k,l), which come from the
 * members Vertex::index, Edge::index, and Face::index.  Note that
 * the ordering of these indices is important because it determines
 * the orientation of the new quads; it is also important to avoid
 * "bowties."  For instance, (l,k,j,i) has the opposite orientation
 * of (i,j,k,l), and if (i,j,k,l) is a proper quad, then (i,k,j,l)
 * will look like a bowtie.
 */
void HalfedgeMesh::buildSubdivisionFaceList(vector<vector<Index> >& subDFaces) {
  // TODO This routine is perhaps the most tricky step in the construction of
  // a subdivision mesh (second, perhaps, to computing the actual Catmull-Clark
  // vertex positions).  Basically what you want to do is iterate over faces,
  // then for each for each face, append N quads to the list (where N is the
  // degree of the face).  For this routine, it may be more convenient to simply
  // append quads to the end of the list (rather than allocating it ahead of
  // time), though YMMV.  You can of course iterate around a face by starting
  // with its first halfedge and following the "next" pointer until you get
  // back to the beginning.  The tricky part is making sure you grab the right
  // indices in the right order---remember that there are indices on vertices,
  // edges, AND faces of the original mesh.  All of these should get used.  Also
  // remember that you must have FOUR indices per face, since you are making a
  // QUAD mesh!

  // TODO iterate over faces
  // TODO loop around face
  // TODO build lists of four indices for each sub-quad
  // TODO append each list of four indices to face list
  showError("buildSubdivisionFaceList() not implemented.");
}

FaceIter HalfedgeMesh::bevelVertex(VertexIter v) {
  // TODO This method should replace the vertex v with a face, corresponding to
  // a bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelVertexComputeNewPositions (which you also have to
  // implement!)

  showError("bevelVertex() not implemented.");
  return facesBegin();
}

FaceIter HalfedgeMesh::bevelEdge(EdgeIter e) {
  // TODO This method should replace the edge e with a face, corresponding to a
  // bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelEdgeComputeNewPositions (which you also have to
  // implement!)

  showError("bevelEdge() not implemented.");
  return facesBegin();
}

FaceIter HalfedgeMesh::bevelFace(FaceIter f) {
  // TODO This method should replace the face f with an additional, inset face
  // (and ring of faces around it), corresponding to a bevel operation. It
  // should return the new face.  NOTE: This method is responsible for updating
  // the *connectivity* of the mesh only---it does not need to update the vertex
  // positions.  These positions will be updated in
  // HalfedgeMesh::bevelFaceComputeNewPositions (which you also have to
  // implement!)

  showError("bevelFace() not implemented.");
  return facesBegin();
}


void HalfedgeMesh::bevelFaceComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double normalShift,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled face.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the originalVertexPositions array) to compute an offset vertex
  // position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); hs++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //

}

void HalfedgeMesh::bevelVertexComputeNewPositions(
    Vector3D originalVertexPosition, vector<HalfedgeIter>& newHalfedges,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled vertex.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., hs.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.

}

void HalfedgeMesh::bevelEdgeComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled edge.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); i++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //

}

void HalfedgeMesh::splitPolygons(vector<FaceIter>& fcs) {
  for (auto f : fcs) splitPolygon(f);
}

void HalfedgeMesh::splitPolygon(FaceIter f) {
  // TODO: (meshedit) 
  // Triangulate a polygonal face
  showError("splitPolygon() not implemented.");
}

EdgeRecord::EdgeRecord(EdgeIter& _edge) : edge(_edge) {
  // TODO: (meshEdit)
  // Compute the combined quadric from the edge endpoints.
  // -> Build the 3x3 linear system whose solution minimizes the quadric error
  //    associated with these two endpoints.
  // -> Use this system to solve for the optimal position, and store it in
  //    EdgeRecord::optimalPoint.
  // -> Also store the cost associated with collapsing this edg in
  //    EdgeRecord::Cost.
}

void MeshResampler::upsample(HalfedgeMesh& mesh) {
  list<FaceIter> faces;
  for (auto fit = mesh.facesBegin(); fit != mesh.facesEnd(); fit++) {
    faces.push_back(fit);
  }
  upsampleSelectedFace(mesh, faces);
}

void MeshResampler::upsampleSelectedFace(HalfedgeMesh& mesh, list<FaceIter>& _faces) {
  set<VertexIter> vertices;
  set<EdgeIter> edges;
  set<FaceIter> faces;

  copy(_faces.begin(), _faces.end(), std::inserter(faces, faces.begin()));

  for (auto f : faces) {
    auto hBegin = f->halfedge();
    auto hIter = hBegin;
    do {
      edges.insert(hIter->edge());
      vertices.insert(hIter->vertex());
      hIter = hIter->next();
    } while (hIter != hBegin);
  }

  cout << "upsampleSelectedFace begin" << endl;
  cout << "--------------------------" << endl;
  cout << "selected faces: " << faces.size() << endl;
  cout << "selected vertices: " << vertices.size() << endl;
  cout << "selected edges: " << edges.size() << endl;
  cout << "--------------------------" << endl;

  //for (auto vit = mesh.verticesBegin(); vit != mesh.verticesEnd(); vit++) {
  for (auto vit: vertices) {
    vit->isNew = false;
    if (!vit->isBoundary()) {

      auto he = vit->halfedge();
      Size k = vit->degree();
      float beta = (k == 3 ? 3.0 / 16.0 : 3.0 / (8.0 * k));
      vit->newPosition = (1.0 - double(k) * beta) * vit->position;

      map<HalfedgeIter, double> N;

      vit->getNeighborhood(N);
      N.erase(vit->halfedge()); // remove this vertex

      for (auto nbr : N) {
        vit->newPosition += beta * nbr.first->vertex()->position;
      }

    }
    else {
      auto nit = vit->halfedge()->twin();
      auto a = nit->vertex()->position;
      while (!nit->isBoundary()) {
        nit = nit->next()->twin();
      }
      auto b = nit->vertex()->position;
      vit->newPosition = (a + b) / 8.0 + (3.0 / 4.0) * vit->position;
    }
  }

  // Next, compute the updated vertex positions associated with edges.
  //for (auto fit = mesh.facesBegin(); fit != mesh.facesEnd(); fit++) {
  for (auto eit: edges) {
    if (!eit->isBoundary()) {
      Vector3D v0 = eit->halfedge()->vertex()->position;
      Vector3D v1 = eit->halfedge()->twin()->vertex()->position;
      Vector3D v2 = eit->halfedge()->next()->twin()->vertex()->position;
      Vector3D v3 = eit->halfedge()->twin()->next()->twin()->vertex()->position;
      eit->newPosition = v0 * 3.0 / 8.0 + v1 * 3.0 / 8.0 + v2 * 1.0 / 8.0 + v3 * 1.0 / 8.0;
    }
    else {
      eit->newPosition = eit->centroid();
    }
  }

  // Next, we're going to split every edge in the mesh, in any order.  For
  // future
  // reference, we're also going to store some information about which
  // subdivided
  // edges come from splitting an edge in the original mesh, and which edges are
  // new.
  // In this loop, we only want to iterate over edges of the original
  // mesh---otherwise,
  // we'll end up splitting edges that we just split (and the loop will never
  // end!)

  // copy original edge to a linked-list
  list<EdgeIter> oldEitList;
  list<VertexIter> newVertices;
  list<EdgeIter> newEdges;
  list<FaceIter> newFaces;

  for (auto eit: edges) {
    eit->isNew = false;
    oldEitList.push_back(eit);
  }

  for (auto eit: oldEitList) {
    auto m = mesh.splitEdge(eit, newVertices, newEdges, newFaces);
    m->isNew = true;
    m->newPosition = eit->newPosition;
    vertices.insert(m);

    auto newHalfedgeIt = m->halfedge();

    newHalfedgeIt->edge()->isNew = false;
    newHalfedgeIt->twin()->next()->edge()->isNew = true;
    newHalfedgeIt->twin()->next()->twin()->next()->edge()->isNew = false;
    newHalfedgeIt->next()->next()->edge()->isNew = true;

    edges.insert(newHalfedgeIt->twin()->next()->edge());
    edges.insert(newHalfedgeIt->twin()->next()->twin()->next()->edge());
    edges.insert(newHalfedgeIt->next()->next()->edge());
  }

  // Finally, flip any new edge that connects an old and new vertex.
  //for (auto eit = mesh.edgesBegin(); eit != mesh.edgesEnd(); eit++) {
  for (auto eit: edges) {
    if (eit->halfedge()->vertex()->isNew != eit->halfedge()->twin()->vertex()->isNew && eit->isNew) {
      mesh.flipEdge(eit);
    }
  }

  // Copy the updated vertex positions to the subdivided mesh
  //for (auto vit = mesh.verticesBegin(); vit != mesh.verticesEnd(); vit++) {
  for (auto vit :vertices) {
    vit->position = vit->newPosition;
  }

  copy(newVertices.begin(), newVertices.end(), std::inserter(vertices, vertices.end()));
  copy(newEdges.begin(), newEdges.end(), std::inserter(edges, edges.end()));
  copy(newFaces.begin(), newFaces.end(), std::inserter(faces, faces.end()));

  // clear alreadySplitted flag
//for (auto fit = mesh.facesBegin(); fit != mesh.facesEnd(); fit++) {
  for (auto fit : faces) {
    fit->alreadySplitted = false;
  }

  // fix halfedge at border
  for (auto eit : edges) {
    if (eit->isBoundary()) {
      auto he = eit->halfedge()->twin();
      auto v = he->vertex();
      if (v->halfedge() != he) {
        v->halfedge() = he;
      }
    }
  }

  cout << "--------------------------" << endl;
  cout << "selected faces: " << faces.size() << endl;
  cout << "selected vertices: " << vertices.size() << endl;
  cout << "selected edges: " << edges.size() << endl;
  cout << "--------------------------" << endl;
  cout << "upsampleSelectedFace end" << endl;

}

void MeshResampler::downsample(HalfedgeMesh& mesh) {
  // TODO: (meshEdit)
  // Compute initial quadrics for each face by simply writing the plane equation
  // for the face in homogeneous coordinates. These quadrics should be stored
  // in Face::quadric
  // -> Compute an initial quadric for each vertex as the sum of the quadrics
  //    associated with the incident faces, storing it in Vertex::quadric
  // -> Build a priority queue of edges according to their quadric error cost,
  //    i.e., by building an EdgeRecord for each edge and sticking it in the
  //    queue.
  // -> Until we reach the target edge budget, collapse the best edge. Remember
  //    to remove from the queue any edge that touches the collapsing edge
  //    BEFORE it gets collapsed, and add back into the queue any edge touching
  //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
  //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
  //    top of the queue.
  showError("downsample() not implemented.");
}

void MeshResampler::resample(HalfedgeMesh& mesh) {
  // TODO: (meshEdit)
  // Compute the mean edge length.
  // Repeat the four main steps for 5 or 6 iterations
  // -> Split edges much longer than the target length (being careful about
  //    how the loop is written!)
  // -> Collapse edges much shorter than the target length.  Here we need to
  //    be EXTRA careful about advancing the loop, because many edges may have
  //    been destroyed by a collapse (which ones?)
  // -> Now flip each edge if it improves vertex degree
  // -> Finally, apply some tangential smoothing to the vertex positions
  showError("resample() not implemented.");
}

}  // namespace CMU462
