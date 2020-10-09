/**
 * This file contains code snippets from
 * https://doc.cgal.org/latest/Point_set_processing_3/Point_set_processing_3_2read_ply_points_with_colors_example_8cpp-example.html and
 * https://doc.cgal.org/latest/Point_set_processing_3/Point_set_processing_3_2write_ply_points_example_8cpp-example.html
 */

// Starndard headers.
#include <array>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// System headers and third-parties.

// CGAL headers.
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/write_ply_points.h>

// Project headers.
#include "common/Filesystem.hpp"

// typedefs for CGAL.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel_t;
typedef Kernel_t::Point_3  Point_t;
typedef Kernel_t::Vector_3 Vector_t;
typedef std::array< unsigned char, 3 > Color_t;

typedef std::tuple< Point_t, Vector_t, Color_t > P_t;
typedef CGAL::Nth_of_tuple_property_map<0, P_t> PointMap_t;
typedef CGAL::Nth_of_tuple_property_map<1, P_t> NormalMap_t;
typedef CGAL::Nth_of_tuple_property_map<2, P_t> ColorMap_t;
typedef std::vector< P_t > PC_t;

static PC_t read_ply_point_cloud( const std::string& fn ) {
    std::ifstream ifs(fn);
    if (!ifs) {
        std::stringstream ss;
        ss << "Cannot open " << fn << " for writing. ";
        throw std::runtime_error( ss.str() );
    }

    PC_t cloud;

    if ( !CGAL::read_ply_points_with_properties(
            ifs,
            std::back_inserter(cloud),
            CGAL::make_ply_point_reader( PointMap_t() ),
            CGAL::make_ply_normal_reader( NormalMap_t() ),
            std::make_tuple( ColorMap_t(), CGAL::Construct_array(), // The number of the PLY_property<>s must be the same with the definition of Color_t.
                             CGAL::PLY_property<unsigned char>("red"),
                             CGAL::PLY_property<unsigned char>("green"),
                             CGAL::PLY_property<unsigned char>("blue") ) ) ) {
        ifs.close();
        std::stringstream ss;
        ss << "Read from PLY file " << fn << " failed. ";
        throw std::runtime_error( ss.str() );
    }

    ifs.close();

    return cloud;
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

static void write_ply_point_clouds( const std::string& fn, const PC_t& cloud, bool flagBinary=true ) {
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
            CGAL::make_ply_point_writer(PointMap_t()),
            CGAL::make_ply_normal_writer(NormalMap_t()),
            std::make_tuple( ColorMap_t(),
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

static void print_usage_message() {
    std::cout << "Usage: \n" << "\tReadPLYPointCloud <input PLY filename> <output PLY filename>\n\n";
}

int main(int argc, char** argv) {
    std::cout << "Hello, ReadPLYPointCloud!\n";
    if ( argc < 3 ) {
        print_usage_message();
        return -1;
    }

    std::string inFn = argv[1];
    std::string outFn = argv[2];
    std::cout << "Read " << inFn << "\n";

    auto cloud = read_ply_point_cloud(inFn);
    std::cout << inFn << " contains " << cloud.size() << " points. \n";

    // Prepare the output directory.
    common::test_directory_by_filename(outFn);

    // The output PLY file will be larger than the original one.
    // Possible reason is the current CGAL kernel uses double-precision floating point number.
    write_ply_point_clouds( outFn, cloud );
    std::cout << "Write the point cloud to " << outFn << "\n";

    return 0;
}
