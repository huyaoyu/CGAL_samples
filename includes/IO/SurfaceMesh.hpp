//
// Created by yaoyu on 10/30/20.
//

#ifndef INCLUDES_IO_SURFACEMESH_HPP
#define INCLUDES_IO_SURFACEMESH_HPP

#include <fstream>
#include <string>

#include <CGAL/Surface_mesh.h>

template < typename MeshT >
void read_surface_mesh( const std::string& fn, MeshT& mesh) {
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

template < typename MeshT >
void write_surface_mesh( const std::string& fn, const MeshT& mesh, bool flagBinary=true ) {
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

#endif //INCLUDES_IO_SURFACEMESH_HPP
