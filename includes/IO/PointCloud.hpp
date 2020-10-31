//
// Created by yaoyu on 10/30/20.
//

#ifndef INCLUDES_IO_POINTCLOUD_HPP
#define INCLUDES_IO_POINTCLOUD_HPP

#include <fstream>
#include <iostream>
#include <string>

#include <CGAL/IO/write_ply_points.h>

template < typename PointT >
void write_point_cloud(
        const std::string& fn,
        const std::vector<PointT>& points,
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
            CGAL::parameters::point_map( CGAL::Identity_property_map<PointT>() ) ) ) {
        ofs.close();
        std::stringstream ss;
        ss << "CGAL::write_ply_points() failed. ";
        throw std::runtime_error( ss.str() );
    }

    ofs.close();
}

#endif //INCLUDES_IO_POINTCLOUD_HPP
