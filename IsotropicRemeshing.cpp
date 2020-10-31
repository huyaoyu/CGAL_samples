//
// Created by yaoyu on 10/20/20.
//

#include <iostream>
#include <string>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/repair.h>

// Local includes.
#include "common/Filesystem.hpp"
#include "common/ScopeTimer.hpp"
#include "IO/SurfaceMesh.hpp"

// Namespace.
namespace pmp = CGAL::Polygon_mesh_processing;

// CGAL typedefs.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3                                   Point_t;
typedef CGAL::Surface_mesh<Point_t>                         Mesh_t;

typedef boost::graph_traits< Mesh_t >::vertex_descriptor   DescVertex_t;
typedef boost::graph_traits< Mesh_t >::face_descriptor     DescFace_t;
typedef boost::graph_traits< Mesh_t >::edge_descriptor     DescEdge_t;
typedef boost::graph_traits< Mesh_t >::halfedge_descriptor DescHalfedge_t;

struct halfedge2edge
{
    halfedge2edge(const Mesh_t &m, std::vector<DescEdge_t> &edges)
            : m_mesh(m), m_edges(edges)
    {}
    void operator()(const DescHalfedge_t &h) const
    {
        m_edges.push_back(edge(h, m_mesh));
    }

    const Mesh_t& m_mesh;
    std::vector<DescEdge_t>& m_edges;
};

static void isotropic_remesh( Mesh_t &mesh, double targetLength, int iters ) {
    FUNCTION_SCOPE_TIMER

    typedef boost::property_map< Mesh_t, CGAL::face_patch_id_t<int> >::type PatchIDPMap_t;

    PatchIDPMap_t fPMap = get( CGAL::face_patch_id_t<int>(), mesh );
    bool fPMapValid = false;
    {
        for ( DescFace_t f : faces(mesh) ) {
            if ( get(fPMap, f) != 1 ) {
                fPMapValid = true;
                break;
            }
        }
    }

    std::cout << "fPMapValid = " << fPMapValid << "\n";

    // Border.
    std::cout << "Split border...\n";
    std::vector<DescEdge_t> border;
    pmp::border_halfedges(faces(mesh),
                          mesh,
                          boost::make_function_output_iterator(halfedge2edge(mesh, border)));
    pmp::split_long_edges(border, targetLength, mesh);

    // Perform remeshing.
    std::cout << "Start remeshing... \n";
    pmp::isotropic_remeshing(
            faces( mesh ),
            targetLength,
            mesh,
            pmp::parameters::number_of_iterations(iters)
                    .protect_constraints(true) );
}

static void remove_self_intersection_and_duplicate( Mesh_t &mesh ) {
    std::cout << "Collecting intersected faces. \n";
    std::vector< std::pair< DescFace_t , DescFace_t > > intersectedFacePairs;
    pmp::self_intersections( mesh, std::back_inserter( intersectedFacePairs ) );

    std::unordered_set<DescFace_t , CGAL::Handle_hash_function> intersectedFaceSet;
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
    std::vector< std::vector< DescVertex_t > > duplicatedVertices;
    int newVerticesNB = pmp::duplicate_non_manifold_vertices(
            mesh, CGAL::parameters::output_iterator( std::back_inserter( duplicatedVertices ) ) );
    std::cout << "newVerticesNB = " << newVerticesNB << "\n";
}

int main( int argc, char** argv ) {
    std::cout << "Hello, IsotropicRemeshing! \n";

    assert ( argc >= 3 );

    std::string inFn  = argv[1];
    std::string outFn = argv[2];

    // Prepare the output directory.
    common::test_directory_by_filename( outFn );

    // Read the input mesh.
    Mesh_t mesh;
    read_surface_mesh(inFn, mesh);

    // Check the mesh.
    if ( !CGAL::is_valid_polygon_mesh(mesh) ) {
        std::stringstream ss;
        ss << "Not a valid mesh. ";
        throw std::runtime_error( ss.str() );
    }

    // Remesh.
    isotropic_remesh( mesh, 0.05, 3 );

    // Repair.
    remove_self_intersection_and_duplicate( mesh );

    // Save the mesh.
    write_surface_mesh( outFn, mesh );

    return 0;
}