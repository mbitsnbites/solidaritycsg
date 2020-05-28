# ![Logo](media/SolidarityCSG-logo.png) SolidarityCSG

SolidarityCSG is a library for quick and robust constructive solid geometry (CSG) operations.

As input and output, the library supports triangle geometries (meshes), and internally the library uses a voxelized representation of the volumes.

## Building

Use CMake to build the library, tools and docs:

```bash
mkdir out && cd out
cmake -G Ninja ..
ninja
ninja doc
```

## License

The library is released uneder the GNU LESSER GENERAL PUBLIC LICENSE. See [COPYING.LESSER](COPYING.LESSER) and [COPYING](COPYING) for more information.
