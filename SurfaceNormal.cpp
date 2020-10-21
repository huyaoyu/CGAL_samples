//
// Created by yaoyu on 10/19/20.
//

#include <fstream>
#include <iostream>
#include <string>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/repair.h>

// Project headers.
#include "common/Filesystem.hpp"

// Macros.
#define FUNC_NAME_STR \
    __FUNCTION__ << ": "

#define NEW_LINE \
    std::cout << "\n";

// Namespace.
namespace pmp = CGAL::Polygon_mesh_processing;

// typedefs for CGAL.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3           Point_t;
typedef Kernel_t::Vector_3          Vector_t;
typedef CGAL::Surface_mesh<Point_t> Mesh_t;

typedef boost::graph_traits<Mesh_t>::vertex_descriptor VertexDesc_t;
typedef boost::graph_traits<Mesh_t>::face_descriptor   FaceDesc_t;

// Global constants.
const std::string PM_FACE_NORMAL   = "f:normals";
const std::string PM_VERTEX_NORMAL = "v:normals";

static void read_surface_mesh( const std::string& fn, Mesh_t& mesh) {
    std::ifstream ifs(fn);
    if ( !ifs ) {
        std::stringstream ss;
        ss << "Failed to open " << fn << " for reading. ";
        throw std::runtime_error( ss.str() );
    }

    if ( !CGAL::read_ply( ifs, mesh ) ) {
        ifs.close();
        std::stringstream ss;
        ss << "Read " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ifs.close();
}

static void estimate_face_normal(Mesh_t &mesh) {
    auto resAddPM = mesh.add_property_map<FaceDesc_t, Vector_t>(PM_FACE_NORMAL, CGAL::NULL_VECTOR);
    if ( resAddPM.second )
        std::cout << FUNC_NAME_STR << "Create surface normals. \n";
    else
        std::cout << FUNC_NAME_STR << "Retrieve surface normals. \n";

    auto faceNormalMap = resAddPM.first;
    pmp::compute_face_normals( mesh, faceNormalMap );
}

static void estimate_vertex_normal(Mesh_t &mesh) {
    auto resAddPM = mesh.add_property_map<VertexDesc_t , Vector_t>(PM_VERTEX_NORMAL, CGAL::NULL_VECTOR);
    if ( resAddPM.second )
        std::cout << FUNC_NAME_STR << "Create vertex normals. \n";
    else
        std::cout << FUNC_NAME_STR << "Retrieve vertex normals. \n";

    auto vertexNormalMap = resAddPM.first;
    pmp::compute_vertex_normals( mesh, vertexNormalMap );
}

int main( int argc, char** argv ) {
    std::cout << "Hello, SurfaceNormal! \n";

    std::string inFn = argv[1];

    // Read the mesh.
    Mesh_t mesh;
    read_surface_mesh( inFn, mesh );

    // Estimate the surface normal.
    std::cout << FUNC_NAME_STR << "Estimate face normals. \n";
    estimate_face_normal( mesh );

    // Show sample surface normal.
    auto faceNormalMap = mesh.property_map<FaceDesc_t, Vector_t>(PM_FACE_NORMAL).first;
    auto iterFN = faceNormalMap.begin();
    for ( int i = 0; i < 10; ++i, ++iterFN ) {
        std::cout << "*iterFN = " << *iterFN << "\n";
    }

    NEW_LINE

    // Estimate the vertex normal.
    std::cout << FUNC_NAME_STR << "Estimate vertex normals. \n";
    estimate_vertex_normal( mesh );

    // Show sample vertex normals.
    auto vertexNormalMap = mesh.property_map<VertexDesc_t, Vector_t>(PM_VERTEX_NORMAL).first;
    auto iterVN = vertexNormalMap.begin();

    std::cout << "Sequentially list the sample vertex normals: \n";
    for ( int i = 0; i < 10; ++i, ++iterVN ) {
        std::cout << "*iterVN = " << *iterVN << "\n";
    }

    std::cout << "List the sample vertex normals according to the vertex descriptors. \n";
    int i = 0;
    for ( const auto& v : mesh.vertices() ) {
        if ( 10 == i ) break;
        std::cout << "vertexNormalMap[v] = " << vertexNormalMap[v] << "\n";
        i++;
    }

    NEW_LINE

    // Check and remove isolated vertices.
    if ( CGAL::is_valid_polygon_mesh( mesh ) ) {
        std::cout << "The mesh is valid. \n";
    } else {
        std::cout << "The mesh is invalid. \n";
    }

    auto nIsolatedVertices = pmp::remove_isolated_vertices( mesh );
    std::cout << "nIsolatedVertices = " << nIsolatedVertices << "\n";

    NEW_LINE

    // Estimate the vertex normals again.
    std::cout << FUNC_NAME_STR << "Estimate the vertex normal again. \n";
    estimate_vertex_normal( mesh );

    // Show sample vertex normals.
    std::cout << "Sequentially list the sample vertex normals: \n";
    vertexNormalMap = mesh.property_map<VertexDesc_t, Vector_t>(PM_VERTEX_NORMAL).first;
    iterVN = vertexNormalMap.begin();
    for ( int i = 0; i < 10; ++i, ++iterVN ) {
        std::cout << "*iterVN = " << *iterVN << "\n";
    }

    std::cout << "List the sample vertex normals according to the vertex descriptors. \n";
    i = 0;
    for ( const auto& v : mesh.vertices() ) {
        if ( 10 == i ) break;
        std::cout << "vertexNormalMap[v] = " << vertexNormalMap[v] << "\n";
        i++;
    }

    return 0;
}