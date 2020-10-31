//
// Created by yaoyu on 10/30/20.
//

#ifndef INCLUDES_IO_PLAINMATRIX_HPP
#define INCLUDES_IO_PLAINMATRIX_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <Eigen/Dense>

template < typename ScalarT >
void read_matrix( const std::string& fn, Eigen::MatrixX<ScalarT>& mat,
                  int rows=0, int cols=0, const std::string& delimiter=" ") {
    // Entry check and set freeSize flag.
    assert(rows >= 0);
    assert(cols >= 0);

    bool freeSize = true;

    if ( ( rows == 0 && cols >  0 ) ||
         ( rows >  0 && cols == 0 ) ) {
        std::stringstream ss;
        ss << "Wrong rows and cols. Only one of rows and cols is zero is not permitted. ";
        throw std::runtime_error( ss.str() );
    } else if ( rows > 0 and cols > 0 ) {
        freeSize = false;
    }

    if ( !freeSize ) mat.resize(rows, cols);

    // Loop over the file.
    std::string line;
    std::ifstream ifs(fn);
    std::size_t count = 0;
    std::vector<std::vector<ScalarT>> raw;
    if ( ifs && ifs.is_open() ) {
        while( std::getline(ifs, line) ) {
            boost::algorithm::trim(line);

            if ( line.empty() ) {
                continue;
            }

            if ( !freeSize && count == rows ) {
                std::stringstream ss;
                ss << "To many rows. Expecting " << rows << " rows. ";
                throw( std::runtime_error( ss.str() ) );
            }

            std::vector<std::string> splitStrings;

            boost::split( splitStrings, line, boost::is_any_of(delimiter) );

            if ( freeSize && count == 0 ) {
                    // First row determines the columns.
                    cols = splitStrings.size();
            }

            if ( splitStrings.size() != cols ) {
                std::stringstream ss;
                ss << "Line " << count << " contains tokens other than " << cols << ". "
                   << "Line: " << line;
                throw( std::runtime_error( ss.str() ) );
            }

            if ( freeSize ) {
                std::vector<ScalarT> row;
                for ( auto& s : splitStrings ) {
                    std::stringstream ss( s );
                    ScalarT number;
                    ss >> number;
                    row.push_back(number);
                }
                raw.push_back( std::move(row) );
            } else {
                for ( int i = 0; i < splitStrings.size(); ++i ) {
                    std::stringstream ss( splitStrings[i] );
                    ScalarT number;
                    ss >> number;
                    mat(count, i) = number;
                }
            }

            count++;
        }
    } else {
        std::stringstream ss;
        ss << "File " << fn << " not opened.";
        throw(std::runtime_error(ss.str()));
    }

    ifs.close();

    if ( freeSize ) {
        // Convert the raw data to Eigen matrix.
        mat.resize( count, cols );

        for ( int i = 0; i < count; i++ ) {
            for ( int j = 0; j < cols; j++ ) {
                mat(i, j) = raw[i][j];
            }
        }
    }
}

#endif //INCLUDES_IO_PLAINMATRIX_HPP
