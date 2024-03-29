# 3D roof and building reconstruction using photogrammetric DSM

This project contains sample Bash and Python scripts along with C++ CGAL executables which perform the following tasks:
* Download [IGNF](https://www.ign.fr/institut/identity-card) [DSM](https://geoservices.ign.fr/actualites/2020-12-10-MNT-MNS) and [BDTOPO](https://geoservices.ign.fr/documentation/donnees/vecteur/bdtopo) building footprint for a given IGNF building identifier
* Crop DSM using BDTOPO footprint (including an optional buffer)
* Transform DSM extract into a point cloud or a mesh supporting different options: normals calculation, walls and floor creation using [BDTOPO](https://geoservices.ign.fr/documentation/donnees/vecteur/bdtopo), axis reordering, local reference coordinates
* Run different CGAL components in order to detect and reconstruct roof panels or building. Among them:
    * [Efficient RANSAC](https://doc.cgal.org/latest/Shape_detection/index.html#title1)
    * [Region Growing](https://doc.cgal.org/latest/Shape_detection/index.html#title10)
    * [Edge Collapse](https://doc.cgal.org/latest/Surface_mesh_simplification/index.html)
    * [Polygonal Surface Reconstruction](https://doc.cgal.org/latest/Polygonal_surface_reconstruction/index.html)


## Building this project

The following instructions have been tested on Ubuntu 22.04 and 20.04

First clone this project

```shell
git clone https://github.com/ignfab/building-roof-pipeline.git
```

### Install required packages

```shell
sudo add-apt-repository ppa:ubuntugis/ppa
sudo apt-get update && sudo apt-get install python3-pip python3-venv python3-dev libgdal-dev gdal-bin gettext-base build-essential wget m4 xz-utils libssl-dev libtbb-dev libreadline-dev pkg-config liblapack-dev libgsl-dev gfortran libopenblas-dev libgsl-dev libcliquer-dev libopenmpi-dev
```

### Build CGAL components executables

1. Enter the projet directory and create download and compilation scripts based on versions defined in `cpplibs_version.cfg` by using the following command:
   ```shell
   cd building-roof-pipeline/
   bash patch_scripts.sh
   ```
   This script copies scripts templates from `script_templates/` directory and replace version numbers using `envsubst` available in `gettext-base` package.

2. Run the script responsible for downloading and compiling the C++ libraries needed for building CGAL components:
   ```shell
   bash dl_and_build_cpplibs.sh -dc
   ```
   See `bash dl_and_build_cpplibs.sh -h` for more information. The downloaded and compiled libraries are available in the `cpplibs/` directory.

4. Run the script responsible for building CGAL components:
   ```shell
   bash build_cgal_components.sh
   ``` 
   The CGAL executables are available in the `cmake-build/` directory

### Create venv

The prefered solution for running the Python scripts is to use virtualenv.  

Create a virtual environment: 

```shell
python3 -m venv venv/roof # create venv
source venv/roof/bin/activate # activate venv
```

### Install Python dependencies with venv

With `roof` virtual environment activated simply run

```shell
(roof) python3 -m pip install -r requirements.txt
```

### Install GDAL Python with venv

Then install GDAL Python. 

```shell
(roof) python3 -m pip install GDAL==$(gdal-config --version) --global-option=build_ext --global-option="-I/usr/include/gdal"
```

## Running this project

A simple Python pipeline script is provided. The following command will test import and transform of a DSM extract. Make sure the `roof` virtual environment is activated.

```shell
(roof) python3 pipeline.py
```

Under the hood, `pipeline.py` uses `.ini` file to configure steps and variables. By default `pipeline.py` uses the `default.ini` file which only performs a sample DSM download and tranformation.  
A sample `full_pipeline.ini` file is provided which contains all the possible steps and variable for the pipeline

```shell
(roof) python3 pipeline.py --file full_pipeline.ini
```

See `python3 pipeline.py --help` to get more information on how to use the pipeline.

## How it works

The `dsm_import.py` script uses [GDAL/OGR Python bindings](https://gdal.org/api/python_bindings.html) and [shapely](https://shapely.readthedocs.io/en/stable/manual.html).  
The `dsm_transform.py` script uses numpy extensively to transform the downloaded DSM extract into a point cloud or a mesh including normals computation. The scripts uses the idea of sliding window implemented using [numpy broadcasting](https://numpy.org/doc/stable/user/basics.broadcasting.html) to create faces and vertex normals efficiently.  
The CGAL documentation provides all the necessary information to modify or add new behaviours to the `.cpp` files in `cgal_components/`.

## Examples

Original IGNF DSM cropped  
![Original IGNF DSM cropped](pics/cropped_dsm.png)

DSM converted into point cloud with normals  
![DSM converted into point cloud with normals](pics/point_cloud_with_normals.png)

DSM converted into mesh with normals  
![DSM converted into mesh with normals](pics/mesh_with_normals.png)

DSM converted into point cloud with normals + walls and floor from BDTOPO  
![DSM converted into point cloud with normals + walls and floor from BDTOPO](pics/point_cloud_walls_with_normals.png)

CGAL region growing on converted DSM Mesh  
![CGAL region growing on converted DSM Mesh](pics/region_growing_mesh.png)

CGAL Polygonal Surface Reconstruction using DSM converted into point cloud with walls and floor  
![CGAL Polygonal Surface Reconstruction using DSM converted into point cloud with walls and floor](pics/psr_ransac.png)

## Contributors 

[![](https://github.com/esgn.png?size=50)](https://github.com/esgn)
[![](https://github.com/indyteo.png?size=50)](https://github.com/indyteo)
