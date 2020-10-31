# CGAL_samples

This repo contains the code for my blog post [here][BlogPost]. The codes practice the CGAL functionalities include

- Read and write point cloud with PLY format with point properties
- Read and write surface mesh.
- Surface mesh property map and normal estimation.
- Mesh check and repair.
- Advancing front surface reconstruction.
- Isotropic remeshing.
- Surface mesh hole identification and filling.
- Ray shooting to a surface mesh.
- Shape detection from a point cloud.

Please refer to the blog for a more detailed discussion.

[BlogPost]: http://www.huyaoyu.com/technical/2020/10/07/cgal-point-cloud-and-mesh.html

## Prerequisites ##

- Ubuntu 18.04 (my current os, only tested in this)
- Boost
- CGAL 5.1 (need to export an environment variable `CGAL_DIR` to show the location)
- Eigen 3 (the version shipped with Ubuntu 18.04 should work)

## Compilation ##

This repo uses a simple `CMakeLists.txt`. Just build it in the usual way, e.g.

```
mkdir build-release
cd build-release
cmake -DCMAKE_BUILD_TYPE=Release <path to code>
make -j4
```

Note that, it is required to have a environment variable defined with the name of `CGAL_DIR` showing where CGAL is located in the file system.

Enjoy!
