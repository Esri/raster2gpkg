#raster2gpkg

This is a Python based Geoprocessing tool that loads jpeg or png images to a GeoPackage. 

![App](https://raw.github.com/Esri/quickstart-map-js/master/raster2gpkg.png)

## Features

## Instructions
1) Copy the contents of the python directory under the python site-packages directory. Usually C:\Python27\ArcGIS10.3\Lib\site-packages.

The python directory contains:
osgeo
gdal.pt
gdalconst.py
gdalnumeric.py
osr.py

2) Copy gdal18.dll to Desktop10.3\bin

3) In ArcCatalog navigate to MyToolboxes, right click, and create a new toolbox and name it "GeoPackage". Copy the Raster2gpkg tool from the GeoPackageToolbox.tbx\GeoPackage to the GeoPackage toolbox under My Toolboxes.

4) Load the image into a geopackage (.gpkg)
 - Navigate to the target geopackage.
 - Navigate to the image to be loaded.
 - Select the image type. (jpeg, png).
 - Select Ok.

5) In ArcMap bring up the Toolbox.
 - Select the Make Raster Layer tool from the Data Management\Layers and Table Views toolbox.
 - Navigate to the geopackage containing the loaded image and select the image (main.q0513nejpg_tiles), Anything you load will have a "_tiles" on the end of the name.
 - Accept the default raster layer name.
 - Click OK. The python script, "raster2gpkg.py is located in \\jotunheimen\C\gdal2gpkgWorkup. It's also been loaded into the GP tool and is not needed for install.
 
6) Data
example.gpkg - Sample empty geopackage. 
q0513ne.jpg - Sample image for loading.

## Requirements

* ArcGIS Desktop 10.3 with http://support.esri.com/en/downloads/patches-servicepacks/view/productid/66/metaid/2200
* ArcGIS Desktop 10.3.1

## Resources
* [ArcGIS Blog](http://blogs.esri.com/esri/arcgis/)

## Issues

Find a bug or want to request a new feature?  Please let us know by submitting an issue.

## Contributing

Esri welcomes contributions from anyone and everyone. Please see our [guidelines for contributing](https://github.com/esri/contributing).

## Licensing
Copyright 2015 Esri

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

A copy of the license is available in the repository's [license.txt]

[](Esri Tags: ArcGIS Raster GeoPackage GPKG)
[](Esri Language: Python)​
