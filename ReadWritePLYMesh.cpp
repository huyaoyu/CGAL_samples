//
// Created by yaoyu on 10/8/20.
//

#include <fstream>
#include <iostream>
#include <string>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

// Project headers.
#include "common/Filesystem.hpp"
#include "IO/SurfaceMesh.hpp"

// typedefs for CGAL.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3           Point_t;
typedef CGAL::Surface_mesh<Point_t> Mesh_t;

typedef boost::graph_traits<Mesh_t>::vertex_descriptor VertexDesc_t;

int main( int argc, char** argv ) {
    std::cout << "Hello, ReadWritePLYMesh! \n";

    assert( argc > 2 );
    std::string inFn  = argv[1];
    std::string outFn = argv[2];

    // Read the surface mesh.
    Mesh_t mesh;
    read_surface_mesh( inFn, mesh );

    // Show meta-data of the mesh.
    std::cout << "mesh.number_of_vertices()  = " << mesh.number_of_vertices() << "\n";
    std::cout << "mesh.number_of_edges()     = " << mesh.number_of_edges() << "\n";
    std::cout << "mesh.number_of_halfedges() = " << mesh.number_of_halfedges() << "\n";
    std::cout << "mesh.number_of_faces()     = " << mesh.number_of_faces() << "\n";

    // Get the vertex property d.
    Mesh_t::Property_map< VertexDesc_t, float > vPropD;
    bool found = false;
    boost::tie( vPropD, found ) =
            mesh.property_map< VertexDesc_t, float >( "v:d" );
    if ( found ) {
        std::cout << "Found vertex property map v:d. \n";
        // Show the content of the vertex property map.
        for ( const auto& v : mesh.vertices() ) {
            std::cout << vPropD[v] << "\n";
        }
    } else {
        std::cout << "Cannot find vertex property map v:d. \n";
    }

    // Modify the vertex property map v:point.
    // Can also get this property map by mesh.points().
    auto vPropPoint = mesh.property_map<VertexDesc_t, Point_t>("v:point").first;
    for ( const auto& v : mesh.vertices() ) {
        const Point_t& oriPoint = vPropPoint[v];
        vPropPoint[v] = Point_t(
                oriPoint.x(),
                oriPoint.y() + 6,
                oriPoint.z());
    }

    // Write the mesh.
    common::test_directory_by_filename( outFn );
    write_surface_mesh(outFn, mesh, false);
    std::cout << "Write the output mesh to " << outFn << " in ascii format. \n";

    outFn = common::add_prefix_suffix( outFn, "", "_binary" );
    write_surface_mesh(outFn, mesh );
    std::cout << "Write the output mesh to " << outFn << " in binary format. \n";

    return 0;
}