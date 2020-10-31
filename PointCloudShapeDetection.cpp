//
// Created by yaoyu on 10/31/20.
//

#include <array>
#include <iostream>
#include <string>
#include <set>
#include <unordered_set>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/write_ply_points.h>

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>

#include "common/Filesystem.hpp"
#include "common/ScopeTimer.hpp"
#include "IO/SurfaceMesh.hpp"

// Namespace.
namespace pmp = CGAL::Polygon_mesh_processing;

// typedefs for CGAL.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3              Point_t;
typedef Kernel_t::Vector_3             Vector_t;
typedef CGAL::Surface_mesh<Point_t>    Mesh_t;

typedef boost::graph_traits<Mesh_t>::face_descriptor     FaceDesc_t;
typedef boost::graph_traits<Mesh_t>::vertex_descriptor   VertexDesc_t;

constexpr int IDX_POINT  = 0;
constexpr int IDX_NORMAL = 1;
constexpr int IDX_VDESC  = 2;
constexpr int IDX_COLOR  = 3;
typedef std::array< unsigned char, 3 > Color_t;
typedef std::tuple< Point_t, Vector_t, VertexDesc_t, Color_t > MeshPoint_t;
typedef CGAL::Nth_of_tuple_property_map< IDX_POINT,  MeshPoint_t > MP_Point_t;
typedef CGAL::Nth_of_tuple_property_map< IDX_NORMAL, MeshPoint_t > MP_Normal_t;
typedef CGAL::Nth_of_tuple_property_map< IDX_VDESC,  MeshPoint_t > MP_VDesc_t;
typedef CGAL::Nth_of_tuple_property_map< IDX_COLOR,  MeshPoint_t > MP_Color_t;
typedef std::vector< MeshPoint_t > MeshPointCloud_t;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits<
        Kernel_t, MeshPointCloud_t, MP_Point_t, MP_Normal_t > ERTraits_t;
typedef CGAL::Shape_detection::Efficient_RANSAC< ERTraits_t > EfficientRansac_t;
typedef CGAL::Shape_detection::Plane< ERTraits_t >            ERPlane_t;

// Global constants.
static const std::string VERTEX_NORMAL_PROP_NAME = "v:normals";
static const std::string VERTEX_PLANE_PROP_NAME  = "v:plane";

static void estimate_vertex_normal(Mesh_t &mesh) {
    FUNCTION_SCOPE_TIMER
    auto vertexNormals = mesh.add_property_map< VertexDesc_t, Vector_t >( VERTEX_NORMAL_PROP_NAME, CGAL::NULL_VECTOR ).first;
    pmp::compute_vertex_normals( mesh, vertexNormals );
}

static MeshPointCloud_t make_mesh_point_cloud( const Mesh_t& mesh ) {
    FUNCTION_SCOPE_TIMER

    // Vertex point property.
    Mesh_t::Property_map< VertexDesc_t , Point_t > vertexPoints;
    bool found;
    boost::tie( vertexPoints, found ) = mesh.property_map< VertexDesc_t, Point_t >( "v:point" );
    assert( found );

    // Vertex normal property.
    auto vertexNormals = mesh.property_map< VertexDesc_t, Vector_t >( VERTEX_NORMAL_PROP_NAME ).first;

    // Mesh point cloud.
    MeshPointCloud_t meshPC;
    const Color_t white { 255, 255, 255 };
    for ( const VertexDesc_t &vDesc : mesh.vertices() ) {
        meshPC.emplace_back( vertexPoints[vDesc], vertexNormals[vDesc], vDesc, white );
    }

    return meshPC;
}

static void shape_detection( MeshPointCloud_t& meshPC,
                             std::vector<int>& pointPlaneMap,
                             std::vector<std::array<Kernel_t::FT, 4>>& planes,
                             std::vector<std::array<Kernel_t::FT, 3>>& planeCentroids ) {
    FUNCTION_SCOPE_TIMER

    // Shape detection engine.
    EfficientRansac_t er;
    er.set_input( meshPC );
    er.add_shape_factory<ERPlane_t>();

    std::cout << "=== Shape detection. ===\n";
    std::cout << "Pre-processing...\n";
    er.preprocess();

    EfficientRansac_t::Parameters erParams;
    erParams.probability      = 0.05;
    erParams.min_points       = 2000;
    erParams.epsilon          = 0.01;
    erParams.cluster_epsilon  = 0.1;
    erParams.normal_threshold = 0.9;

    std::cout << "Detecting...\n";
    er.detect(erParams);

    EfficientRansac_t::Shape_range erShapes = er.shapes();
    EfficientRansac_t::Shape_range::iterator ersIter = erShapes.begin();

    std::cout << "Assign planes to the points...\n";
    pointPlaneMap.resize( meshPC.size() );
    planes.clear();
    for ( int i = 0; ersIter != erShapes.end(); ersIter++, i++ ) {
        boost::shared_ptr< EfficientRansac_t::Shape > shape = *ersIter;
        auto pPlane = reinterpret_cast<ERPlane_t*>( shape.get() );
        const auto &normal = pPlane->plane_normal();

        planes.push_back( {
              static_cast<Kernel_t::FT>(normal.x()),
              static_cast<Kernel_t::FT>(normal.y()),
              static_cast<Kernel_t::FT>(normal.z()),
              static_cast<Kernel_t::FT>(pPlane->d()) } );

//        std::cout << i << ": " << shape->info() << "\n";

        Kernel_t::FT x = 0, y = 0, z = 0;

        auto ptIdxIter = shape->indices_of_assigned_points().begin();
        while ( ptIdxIter != shape->indices_of_assigned_points().end() ) {
            pointPlaneMap[ *ptIdxIter ] = i;
            const auto& point = std::get<IDX_POINT>( meshPC[*ptIdxIter] );
            x += point.x();
            y += point.y();
            z += point.z();
            ptIdxIter++;
        }

        const int NP = shape->indices_of_assigned_points().size();
        planeCentroids.push_back( { x/NP, y/NP, z/NP } );

        // Show info.
        std::cout << i << ": [ "
                  << planes[i][0] << ", "
                  << planes[i][1] << ", "
                  << planes[i][2] << ", "
                  << planes[i][2] << "]. "
                  << NP << " points. "
                  << "Centroid [ "
                  << planeCentroids[i][0] << ", "
                  << planeCentroids[i][1] << ", "
                  << planeCentroids[i][2] << " ]. \n";
    }
}

struct ColorCircle {
    ColorCircle() {
        // https://en.wikipedia.org/wiki/Web_colors
        colors.push_back( { 255,   0,   0 } ); // Red.
        colors.push_back( { 128,   0,   0 } ); // Maroon.
        colors.push_back( { 255, 255,   0 } ); // Yellow.
        colors.push_back( { 128, 128,   0 } ); // Olive.
        colors.push_back( { 000, 255,   0 } ); // Lime.
        colors.push_back( { 000, 128,   0 } ); // Green.
        colors.push_back( { 000, 255, 255 } ); // Aqua.
        colors.push_back( { 000, 128, 128 } ); // Teal.
        colors.push_back( { 000,   0, 255 } ); // Blue.
        colors.push_back( { 000,   0, 128 } ); // Navy.
        colors.push_back( { 255,   0, 255 } ); // Fuchsia.
        colors.push_back( { 128,   0, 128 } ); // Purple.

        pos = colors.size();
    }

    const Color_t& next() {
        pos++;
        if ( pos >= colors.size() ) pos = 0;
        return colors[pos];
    }

    [[nodiscard]] const Color_t& get(int idx) const {
        assert(idx >= 0);
        return colors[ idx % colors.size() ];
    }

    int pos;
    std::vector< Color_t > colors;
};

static void color_point_cloud( MeshPointCloud_t& meshPC, const std::vector<int>& pointPlaneMap ) {
    ColorCircle cc;
    const int N = meshPC.size();
    for ( int i = 0; i < N; ++i ) {
        std::get<3>( meshPC[i] ) = cc.get( pointPlaneMap[i] );
    }
}

namespace CGAL {
    template < typename T >
    struct Output_rep< ::Color_t, T > {
        const ::Color_t& c;
        static const bool is_specialized = true;

        Output_rep( const ::Color_t& c ) : c{c} {}

        std::ostream& operator() ( std::ostream& out ) const {
            if ( is_ascii( out ) ) {
                out << static_cast<int>(c[0]) << " "
                    << static_cast<int>(c[1]) << " "
                    << static_cast<int>(c[2]);
            } else {
                out.write( reinterpret_cast<const char*>(&c), sizeof(c) );
            }

            return out;
        }
    };
}

static void write_ply_point_clouds(
        const std::string& fn,
        const MeshPointCloud_t& cloud,
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

    if ( !CGAL::write_ply_points_with_properties(
            ofs,
            cloud,
            CGAL::make_ply_point_writer(MP_Point_t()),
            CGAL::make_ply_normal_writer(MP_Normal_t()),
            std::make_tuple( MP_Color_t (),
                             CGAL::PLY_property<unsigned char>("red"),
                             CGAL::PLY_property<unsigned char>("green"),
                             CGAL::PLY_property<unsigned char>("blue") ) ) ) {
        ofs.close();
        std::stringstream ss;
        ss << "Write to " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ofs.close();
}

int main( int argc, char** argv ) {
    std::cout << "Hello, PCShapeDetection! \n";

    assert( argc >= 3 );

    const std::string inFn  = argv[1];
    const std::string outFn = argv[2];

    // Prepare the output directory.
    common::test_directory_by_filename( outFn );

    // Read the mesh.
    Mesh_t mesh;
    read_surface_mesh( inFn, mesh );
    std::cout << "mesh.number_of_vertices() = " << mesh.number_of_vertices() << "\n";
    std::cout << "mesh.number_of_faces() = " << mesh.number_of_faces() << "\n";

    // Estimate the vertex normal.
    estimate_vertex_normal( mesh );

    // Extract points and normals from the surface mesh vertices.
    auto pc = make_mesh_point_cloud( mesh );

    // Detect planes.
    std::vector<int> pointPlaneMap;
    std::vector<std::array<Kernel_t::FT, 4>> planes;
    std::vector<std::array<Kernel_t::FT, 3>> planeCentroids;
    shape_detection( pc, pointPlaneMap, planes, planeCentroids );

    // Color.
    color_point_cloud( pc, pointPlaneMap );

    // Write the point cloud.
    write_ply_point_clouds( outFn, pc );

    return 0;
}