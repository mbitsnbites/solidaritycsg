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

## Using the library

The SolidarityCSG library can be used by applications to perform CSG. See the Doxygen documentation for more details.

## Using the csg tool

A small demonstration tool that uses the SolidarityCSG can be found in the tools directory. After building, it can be found in `out/tools/csg`.

The `csg` tool takes one or more scene description XML files as input:

```bash
$ tools/csg myscene.xml
```

Here is an example XML file:

```xml
<?xml version="1.0" encoding='UTF-8'?>
<csg>
  <settings>
    <resolution x="0.001" y="0.001" z="0.001" />
    <output name="example" type="mesh" format="stl" />
  </settings>
  <volume>
    <difference>
      <box cx="0.0" cy="0.0" cz="0.0" sx="0.1" sy="0.1" sz="0.1" />
      <mesh src="3d-models/bunny.stl" />
    </difference>
  </volume>
</csg>
```

Example outputs:

* 3D model in STL format:
  * `<output name="example" type="mesh" format="stl" />`
* 2D slices (many TGA files):
  * `<output name="example" type="slice" format="tga" />`

Valid CSG operations:

* `<union>`
* `<difference>`
* `<intersection>`

Valid shapes (examples):

* `<sphere cx="0.0" cy="0.0" cz="0.0" r="0.6" />`
* `<box cx="0.0" cy="0.0" cz="0.0" sx="1.0" sy="1.0" sz="1.0" />`
* `<mesh src="3d-models/bunny.stl" />`


## License

The library is released uneder the GNU LESSER GENERAL PUBLIC LICENSE. See [COPYING.LESSER](COPYING.LESSER) and [COPYING](COPYING) for more information.
