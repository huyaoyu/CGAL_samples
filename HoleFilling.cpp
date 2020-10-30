//
// Created by yaoyu on 10/30/20.
//

#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/write_ply_points.h>

#include <CGAL/Handle_hash_function.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

// Project headers.
#include "common/Filesystem.hpp"
#include "common/ScopeTimer.hpp"

namespace pmp = CGAL::Polygon_mesh_processing;

// typedefs for CGAL.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3           Point_t;
typedef CGAL::Surface_mesh<Point_t> Mesh_t;

typedef boost::graph_traits<Mesh_t>::vertex_descriptor   VertexDesc_t;
typedef boost::graph_traits<Mesh_t>::face_descriptor     FaceDesc_t;
typedef boost::graph_traits<Mesh_t>::halfedge_descriptor HalfedgeDesc_t;

typedef std::unordered_set< FaceDesc_t, CGAL::Handle_hash_function > PatchFacets_t;
typedef std::vector< VertexDesc_t > PatchVertices_t;

typedef Mesh_t::Property_map< FaceDesc_t, int > FaceHoleTagProp_t;

// Global constants.
static const std::string HOLE_TAG_PROP_NAME = "HoleTag";

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

static void write_surface_mesh( const std::string& fn, const Mesh_t& mesh, bool flagBinary=true ) {
    std::ofstream ofs;
    if ( flagBinary ) {
        ofs.open(fn, std::ios::binary);
        CGAL::set_binary_mode(ofs);
    } else {
        ofs.open(fn);
    }

    if ( !ofs ) {
        std::stringstream ss;
        ss << "Open " << fn << " for writing failed. ";
        throw std::runtime_error( ss.str() );
    }

    if ( !CGAL::write_ply(ofs, mesh) ) {
        ofs.close();
        std::stringstream ss;
        ss << "Write " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ofs.close();
}

static void fill_holes( Mesh_t& mesh,
                        std::vector<PatchFacets_t>& holePatchFaces,
                        std::vector<PatchVertices_t>& holePatchVertices) {
    FUNCTION_SCOPE_TIMER
    typedef std::unordered_set< FaceDesc_t, CGAL::Handle_hash_function > PatchFacets_t;
    typedef std::vector< VertexDesc_t > PatchVertices_t;

    int nFilled = 0;

    // Fill all holes.
    for ( HalfedgeDesc_t h : halfedges(mesh) ) {
        if ( !is_border(h, mesh) ) {
            continue;
        }

        PatchFacets_t patchFacets;
        PatchVertices_t patchVertices;

        pmp::triangulate_and_refine_hole(
                mesh,
                h,
                std::inserter( patchFacets, patchFacets.begin() ),
                std::back_inserter( patchVertices ),
                pmp::parameters::vertex_point_map( get(CGAL::vertex_point, mesh) )
                    .density_control_factor( 2 )
                    .use_delaunay_triangulation(true)
                    .geom_traits(Kernel_t())
                );

        nFilled++;
        std::cout << "Hole no. " << nFilled << "\n";
        std::cout << "Facets in patch: " << patchFacets.size() << "\n";
        std::cout << "Vertices in patch: " << patchVertices.size() << "\n";

        holePatchFaces.push_back( std::move( patchFacets ) );
        holePatchVertices.push_back( std::move( patchVertices ) );
    }

    // Tag the hole facets as a new property map.
    std::cout << "Create a new face property map for the filled facets identification. \n";
    FaceHoleTagProp_t facetProperties;
    bool created;
    boost::tie(facetProperties, created) =
            mesh.add_property_map<FaceDesc_t, int>(HOLE_TAG_PROP_NAME, -1);
    assert(created);

    // Tag.
    int holeIdx = 0;
    for (auto &facetSet : holePatchFaces) {
        for (auto &facet : facetSet) {
            facetProperties[facet] = holeIdx;
        }
        holeIdx++;
    }
}

static void remesh_facet_set(
        Mesh_t& mesh, std::vector< PatchFacets_t >& facetSetContainer,
        double targetLength=0.1 ) {
    FUNCTION_SCOPE_TIMER

    typedef typename boost::graph_traits<Mesh_t>::face_descriptor FaceDesc_T;

    // Retrieve property map.
    FaceHoleTagProp_t facetProperties;
    bool found;
    boost::tie( facetProperties, found ) =
            mesh.template property_map< FaceDesc_T, int >(HOLE_TAG_PROP_NAME);
    assert( found );

    int idx = 0;
    for ( auto& facetSet : facetSetContainer ) {
        pmp::isotropic_remeshing(
                facetSet,
                targetLength,
                mesh,
                pmp::parameters::number_of_iterations(1)
                        .protect_constraints(false)
                        .face_patch_map( facetProperties )
        );

        std::cout << "Hole index " << idx << " ( " << facetSet.size() << " facets ) remeshed. \n";
        ++idx;
    }
}

static std::vector<Point_t> collect_points_by_facet_tag( Mesh_t &mesh ) {
    FUNCTION_SCOPE_TIMER
    typedef CGAL::Vertex_around_face_iterator<Mesh_t> VertexAroundFaceIter_t;

    // Retrieve the facet property.
    FaceHoleTagProp_t facetProperties;
    bool found;
    boost::tie( facetProperties, found ) =
            mesh.property_map< FaceDesc_t, int >(HOLE_TAG_PROP_NAME);
    assert( found );

    // Collect the vertices into a set.
    std::unordered_set< VertexDesc_t, CGAL::Handle_hash_function > vertexSet;
    VertexAroundFaceIter_t vi, ve;

    for ( auto &facet : mesh.faces() ) {
        if ( facetProperties[facet] < 0 ) {
            // Not originates from a hole area.
            continue;
        }

        for ( boost::tie(vi, ve) = vertices_around_face( mesh.halfedge(facet), mesh ); vi != ve; ++vi ) {
            vertexSet.insert( *vi );
        }
    }

    // Copy the actual points into a container.
    std::vector< Point_t > points;

    if ( vertexSet.empty() ) return points;

    points.reserve( vertexSet.size() );
    auto pointProperties = mesh.points();
    for ( auto& vertex : vertexSet ) {
        points.push_back( pointProperties[vertex] ); // Copy.
    }

    return points;
}

static void write_point_cloud(
        const std::string& fn,
        const std::vector<Point_t>& points,
        bool flagBinary=true ) {
    std::ofstream ofs;

    if ( flagBinary ) {
        ofs.open( fn, std::ios::binary );
        CGAL::set_binary_mode(ofs);
    } else {
        ofs.open( fn );
    }

    if (!ofs) {
        std::stringstream ss;
        ss << "Failed to open " << fn << " for writing. ";
        throw std::runtime_error( ss.str() );
    }

    if ( !CGAL::write_ply_points(
            ofs, points,
            CGAL::parameters::point_map( CGAL::Identity_property_map<Point_t>() ) ) ) {
        ofs.close();
        std::stringstream ss;
        ss << "CGAL::write_ply_points() failed. ";
        throw std::runtime_error( ss.str() );
    }

    ofs.close();
}

int main( int argc, char** argv ) {
    std::cout << "Hello, HoleFilling! \n";

    assert( argc >= 3 );

    std::string inFn   = argv[1];
    std::string outDir = argv[2];

    // Prepare the output directory.
    common::test_directory(outDir);

    // Read the input mesh.
    Mesh_t mesh;
    read_surface_mesh(inFn, mesh);

    // Show some meta data of the mesh.
    std::cout << "mesh.number_of_vertices() = " << mesh.number_of_vertices() << "\n";
    std::cout << "mesh.number_of_faces() = " << mesh.number_of_faces() << "\n";

    // Fill holes.
    std::vector< PatchFacets_t > holePatchFaces;
    std::vector< PatchVertices_t > holePatchVertices;
    std::cout << "Start filling holes. \n";
    fill_holes( mesh, holePatchFaces, holePatchVertices );

    // Write the mesh.
    {
        std::stringstream ss;
        ss << outDir << "/HoleFilled.ply";
        write_surface_mesh( ss.str(), mesh );
    }

    // Remesh the hole areas.
    remesh_facet_set( mesh, holePatchFaces, 0.1 );

    // Save the mesh.
    {
        std::stringstream ss;
        ss << outDir << "/HoleFilledRemeshed.ply";
        write_surface_mesh( ss.str(), mesh );
    }

    // Collect the points.
    auto holePoints = collect_points_by_facet_tag( mesh );

    // Write the points.
    {
        std::stringstream ss;
        ss << outDir << "/FilledPoints.ply";
        write_point_cloud( ss.str(), holePoints );
    }

    return 0;
}