//
// Created by yaoyu on 10/20/20.
//

#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/read_ply_points.h>

#include <CGAL/Advancing_front_surface_reconstruction.h>

// Project headers.
#include "common/Filesystem.hpp"
#include "common/ScopeTimer.hpp"

// typedefs for CGAL.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3           Point_t;
typedef CGAL::Surface_mesh<Point_t> Mesh_t;

// Handy mesh descriptors.
typedef boost::graph_traits<Mesh_t>::vertex_descriptor VertexDesc_t;

// For reading point cloud.
typedef CGAL::Identity_property_map<Point_t> PCPM_t;
typedef std::vector< Point_t > PC_t;

// For surface reconstruction.
typedef std::array< std::size_t, 3 > Facet_t;

// Global constants.
const auto G_PI = boost::math::constants::pi<double>();

struct PriorityRadius {
    double bound;

    explicit PriorityRadius ( double b )
            : bound{b} {}

    template < typename AdvFront_T, typename CellHandle_T >
    double operator () ( const AdvFront_T &adv, CellHandle_T &c, const int &index ) const {
        if ( 0 == bound ) {
            return adv.smallest_radius_delaunay_sphere( c, index );
        }

        double d = 0;
        d = sqrt( CGAL::squared_distance( c->vertex( (index+1)%4 )->point(),
                                          c->vertex( (index+2)%4 )->point() ) );
        if ( d > bound ) return adv.infinity();

        d = sqrt( CGAL::squared_distance( c->vertex( (index+2)%4 )->point(),
                                          c->vertex( (index+3)%4 )->point() ) );
        if ( d > bound ) return adv.infinity();

        d = sqrt( CGAL::squared_distance( c->vertex( (index+1)%4 )->point(),
                                          c->vertex( (index+3)%4 )->point() ) );
        if ( d > bound ) return adv.infinity();

        return adv.smallest_radius_delaunay_sphere( c, index );
    }
};

struct MeshConstructorByPointIndices {
    typedef boost::property_map< Mesh_t, boost::vertex_point_t >::type VPMap_t;

    Mesh_t& mesh;
    VPMap_t vpMap;
    std::vector< VertexDesc_t > vertices;

    template < typename Point_T >
    MeshConstructorByPointIndices( Mesh_t &m, const std::vector<Point_T> &points )
            : mesh{m} {
        vpMap = get(boost::vertex_point, mesh);
        for ( const auto& p : points ) {
            VertexDesc_t v;
            v = add_vertex( mesh );
            vertices.push_back(v);
            put( vpMap, v, p );
        }
    }

    MeshConstructorByPointIndices& operator = ( const Facet_t f ) {
        std::vector< VertexDesc_t > facet(3);

        facet[0] = vertices[ f[0] ];
        facet[1] = vertices[ f[1] ];
        facet[2] = vertices[ f[2] ];

        CGAL::Euler::add_face( facet, mesh );

        return *this;
    }

    MeshConstructorByPointIndices& operator * () { return *this; }
    MeshConstructorByPointIndices& operator ++ () { return *this; }
    MeshConstructorByPointIndices  operator ++ (int) { return *this; }
};

static void reconstruct_surface_mesh(
        const PC_t& points, Mesh_t &mesh,
        double longestEdge=0.1, double radiusRatioBound=5.0, double betaDeg=30.0 ) {
    FUNCTION_SCOPE_TIMER
    std::cout << "Begin reconstruction by AFS. \n";
    PriorityRadius priority(longestEdge);
    MeshConstructorByPointIndices meshConstructor( mesh, points );

    std::cout << "AFS... \n";
    CGAL::advancing_front_surface_reconstruction(
            points.begin(), points.end(),
            meshConstructor, priority,
            radiusRatioBound, betaDeg/180.0*G_PI );
}

static PC_t read_point_cloud( const std::string& fn ) {
    std::ifstream ifs(fn);
    if (!ifs) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for writing. ";
        throw std::runtime_error( ss.str() );
    }

    PC_t cloud;

    if ( !CGAL::read_ply_points(
            ifs,
            std::back_inserter(cloud),
            CGAL::parameters::point_map( PCPM_t() ) ) ) {
        ifs.close();
        std::stringstream ss;
        ss << "Read from PLY file " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ifs.close();

    return cloud;
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

int main( int argc, char** argv ) {
    std::cout << "Hello, AFSReconstruction! \n";

    assert( argc >= 3 );

    std::string inFn  = argv[1];
    std::string outFn = argv[2];

    // Prepare the output directory.
    common::test_directory_by_filename( outFn );

    // Read the point cloud.
    auto cloud = read_point_cloud( inFn );
    std::cout << "cloud.size() = " << cloud.size() << "\n";

    // Reconstruct surface.
    Mesh_t mesh;
    reconstruct_surface_mesh( cloud, mesh, 0.4 );

    // Write the mesh.
    write_surface_mesh( outFn, mesh );

    return 0;
}