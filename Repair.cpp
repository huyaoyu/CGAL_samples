//
// Created by yaoyu on 10/19/20.
//

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/repair.h>

// Project headers.
#include "common/Filesystem.hpp"
#include "IO/SurfaceMesh.hpp"

// Namespace.
namespace pmp = CGAL::Polygon_mesh_processing;

// typedefs for CGAL.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3           Point_t;
typedef Kernel_t::Vector_3          Vector_t;
typedef CGAL::Surface_mesh<Point_t> Mesh_t;

typedef boost::graph_traits<Mesh_t>::vertex_descriptor VertexDesc_t;
typedef boost::graph_traits<Mesh_t>::face_descriptor   FaceDesc_t;

static void remove_self_intersection_and_duplicate( Mesh_t &mesh ) {
    std::cout << "Collecting intersected faces. \n";
    std::vector< std::pair< FaceDesc_t , FaceDesc_t > > intersectedFacePairs;
    pmp::self_intersections( mesh, std::back_inserter( intersectedFacePairs ) );

    std::unordered_set<FaceDesc_t , CGAL::Handle_hash_function> intersectedFaceSet;
    for ( const auto &p : intersectedFacePairs ) {
        intersectedFaceSet.insert( p.first );
        intersectedFaceSet.insert( p.second );
    }

    std::cout << intersectedFaceSet.size() << " faces to be removed. \n";

    std::cout << "Begin removing intersected faces. \n";
    for ( const auto &f : intersectedFaceSet ) {
        CGAL::Euler::remove_face( halfedge(f, mesh), mesh );
    }
    std::cout << "Removal done. \n";

    std::cout << "Begin duplicating non-manifold vertices. \n";
    std::vector< std::vector< VertexDesc_t > > duplicatedVertices;
    int newVerticesNB = pmp::duplicate_non_manifold_vertices(
            mesh, CGAL::parameters::output_iterator( std::back_inserter( duplicatedVertices ) ) );
    std::cout << "newVerticesNB = " << newVerticesNB << "\n";
}

int main( int argc, char** argv ) {
    std::cout << "Hello, Repair! \n";

    assert ( argc >= 3 );

    std::string inFn  = argv[1];
    std::string outFn = argv[2];

    // Prepare the output directory.
    common::test_directory_by_filename( outFn );

    // Read the surface mesh.
    Mesh_t mesh;
    read_surface_mesh( inFn, mesh );

    // Check the validity of the surface mesh as a polygon mesh.
    if ( CGAL::is_valid_polygon_mesh( mesh ) ) {
        std::cout << "The mesh is valid. \n";
    } else {
        std::cout << "The mesh is invalid. \n";
    }

    // Repair.
    remove_self_intersection_and_duplicate( mesh );

    // Write the repaired mesh.
    write_surface_mesh( outFn, mesh );

    return 0;
}