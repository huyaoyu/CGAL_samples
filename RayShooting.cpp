//
// Created by yaoyu on 10/30/20.
//

#include <array>
#include <iostream>
#include <string>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>

#include "common/Filesystem.hpp"
#include "IO/PlainMatrix.hpp"
#include "IO/PointCloud.hpp"
#include "IO/SurfaceMesh.hpp"

namespace pmp = CGAL::Polygon_mesh_processing;

// typedefs for CGAL.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3           Point_t;
typedef Kernel_t::Vector_3          Vector_t;
typedef Kernel_t::Ray_3             Ray_t;
typedef CGAL::Surface_mesh<Point_t> Mesh_t;

typedef boost::graph_traits<Mesh_t>::vertex_descriptor   VertexDesc_t;
typedef boost::graph_traits<Mesh_t>::face_descriptor     FaceDesc_t;

// AABB tree.
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh_t> AABBPrimitive_t;
typedef CGAL::AABB_traits<Kernel_t, AABBPrimitive_t>     AABBTraits_t;
typedef CGAL::AABB_tree<AABBTraits_t>                    AABBTree_t;
typedef boost::optional<AABBTree_t::Intersection_and_primitive_id<Ray_t>::Type> RayIntersection_t;

// Global constants.
static const std::string VERTEX_NORMALS = "v:normals";
static const std::string FACE_NORMALS   = "f:normals";

class Point2MeshProjection {
public:
    typedef std::array<double, 3> PointProxy_t;
public:
    Point2MeshProjection()
            : normalShift{0.001}, initialized{false} {}

    void set_normal_shift(double s) { normalShift = s; }
    double get_normal_shift() const { return normalShift; }

    void initialize_mesh( const std::string &fn );
    bool is_initialized() const { return initialized; }
    std::pair<PointProxy_t, bool> project_single( const Point_t& point, const Vector_t& direction );

private:
    inline void throw_if_not_initialized(const std::string& fn, int lineNo);
    void compute_face_normal();

private:
    double normalShift;
    bool initialized;
    Mesh_t mesh;
    Mesh_t::Property_map< FaceDesc_t, Vector_t > fNormalMap;
    AABBTree_t tree;
};

void Point2MeshProjection::throw_if_not_initialized(
        const std::string& fn, const int lineNo) {
    if ( !initialized ) {
        std::stringstream ss;
        ss << "Point2MeshProjection not initialized yet. "
           << "Entry point: " << fn << "line: " << lineNo << ". ";
        throw std::runtime_error( ss.str() );
    }
}

void Point2MeshProjection::initialize_mesh( const std::string &fn ) {
    if ( initialized ) {
        std::stringstream ss;
        ss << "Already initialized. ";
        throw std::runtime_error( ss.str() );
    }

    // Read the surface mesh and show info.
    read_surface_mesh( fn, mesh );
    std::cout << "sm.number_of_vertices() = " << mesh.number_of_vertices() << "\n";
    std::cout << "sm.number_of_facets()   = " << mesh.number_of_faces() << "\n";

    compute_face_normal();
    tree.insert( faces(mesh).first, faces(mesh).second, mesh );

    initialized = true;
}

void Point2MeshProjection::compute_face_normal() {
    // Add the face normal property to the mesh.
    fNormalMap = mesh.add_property_map< FaceDesc_t, Vector_t >(
            FACE_NORMALS, CGAL::NULL_VECTOR ).first;

    // CGAL makes the face normal pointing outwards.
    pmp::compute_face_normals(mesh, fNormalMap);
}

std::pair<Point2MeshProjection::PointProxy_t, bool>
Point2MeshProjection::project_single( const Point_t& point, const Vector_t& direction ) {
    throw_if_not_initialized( __FILE__, __LINE__ );

    Ray_t ray( point, direction );
    RayIntersection_t intersection = tree.first_intersection( ray );

    PointProxy_t pp { 0, 0, 0 }; // Projected point.
    bool res = false;

    if ( intersection ) {
        if ( boost::get<Point_t>(&( intersection->first )) ) {
            const Point_t *interP = boost::get<Point_t>( &( intersection->first ) );
            pp[0] = static_cast<double>(interP->x());
            pp[1] = static_cast<double>(interP->y());
            pp[2] = static_cast<double>(interP->z());

            // Shift the point along the face normal.
            const Vector_t &normal = fNormalMap[ intersection->second ];
            pp[0] += normalShift * normal.x();
            pp[1] += normalShift * normal.y();
            pp[2] += normalShift * normal.z();

            res = true;
        }
    }

    return { pp, res };
}

template < typename PixelScalarT, typename ParamScalarT, typename OutScalarT >
static void pixel_2_3D_direction( const Eigen::MatrixX<PixelScalarT>& pixels,
                        ParamScalarT fx, ParamScalarT fy,
                        ParamScalarT cx, ParamScalarT cy,
                        Eigen::MatrixX<OutScalarT>& points) {
    points = Eigen::MatrixX<OutScalarT>::Ones( 3, pixels.cols() );

    points( 0, Eigen::all ) = ( ( pixels.row(0).array() - cx ) / fx ).matrix();
    points( 1, Eigen::all ) = ( ( pixels.row(1).array() - cy ) / fy ).matrix();

    points.colwise().normalize();
}

int main( int argc, char** argv ) {
    std::cout << "Hello, RayShooting! \n";

    // ========== Input arguments. =========
    assert( argc >= 5 );
    const std::string inMeshFn      = argv[1];
    const std::string inCamPoseFn   = argv[2];
    const std::string inCamIntFn    = argv[3]; // Camera intrinsic parameters.
    const std::string inPixelCoorFn = argv[4];
    const std::string outFn         = argv[5];

    // Prepare the output directory.
    common::test_directory_by_filename( outFn );

    // ========== Read camera info. ==========
    // Read the camera pose.
    Eigen::MatrixXf camPose;
    read_matrix( inCamPoseFn, camPose );
    std::cout << "camPose = \n" << camPose << "\n";

    // Read the camera intrinsic parameters.
    Eigen::MatrixXf camInt;
    read_matrix( inCamIntFn, camInt );
    std::cout << "camInt = \n" << camInt << "\n";

    // ========== Marked pixels. ==========
    // Read the pixel coordinates.
    Eigen::MatrixXf pixels;
    read_matrix( inPixelCoorFn, pixels, 0, 0, "," );
    std::cout << "pixels: "
              << pixels.rows() << ", "
              << pixels.cols() << " \n";

    pixels.transposeInPlace();

    // Convert the pixel coordinates into 3D point clouds.
    Eigen::MatrixXf directions;
    pixel_2_3D_direction( pixels,
                          camInt(0, 0), camInt(1, 0), camInt(2, 0), camInt(3, 0),
                          directions );

    // Rotate the direction vectors.
    directions = camPose.block( 0, 0, 3, 3 ) * directions.eval();

    // Print some samples.
    std::cout << "directions: "
              << directions.rows() << ", "
              << directions.cols() << " \n";
    std::cout << "directions( Eigen::all, Eigen::seqN(0, 2) ) = \n"
              << directions( Eigen::all, Eigen::seqN(0, 2) ) << "\n";

    // Save the origin point and the directions as global points for visualization.
    Point_t camOrigin( camPose(0, 3), camPose(1, 3), camPose(2, 3) );
    std::vector<Point_t> directionPoints;
    directionPoints.push_back( camOrigin );

    const int N = directions.cols();
    const float scale = 0.05;
    for ( int i = 0; i < N; i++ ) {
        directionPoints.emplace_back(
                directions(0, i) * scale + camOrigin.x(),
                directions(1, i) * scale + camOrigin.y(),
                directions(2, i) * scale + camOrigin.z());
    }

    {
        // Write the visualization points.
        auto parts = common::get_file_parts( outFn );
        std::stringstream ss;
        ss << parts[0] << "/DirectionPoints.ply";
        write_point_cloud( ss.str(), directionPoints );
    }

    // ========== Ray shooting / projection. ==========
    // The mesh projector.
    Point2MeshProjection projector;
    projector.initialize_mesh( inMeshFn );
    projector.set_normal_shift( 0.001 ); // Make the projected points floating on the surface.

    std::vector<Point_t> intersectionPoints;
    int nMiss = 0;
    for ( int i = 0; i < N; i++ ) {
        // Ray shooting direction.
        Vector_t direction( directions(0, i), directions(1, i), directions(2, i) );

        // Shoot the ray.
        auto res = projector.project_single( camOrigin, direction );
        if ( res.second ) {
            intersectionPoints.emplace_back( res.first[0], res.first[1], res.first[2] );
        } else {
            nMiss++;
        }
    }

    std::cout << intersectionPoints.size() << " hits, " << nMiss << " misses. \n";

    // Write the point cloud of the projected points.
    write_point_cloud( outFn, intersectionPoints );

    return 0;
}