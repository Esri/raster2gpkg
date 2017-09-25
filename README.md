#raster2gpkg

This is a Python based Geoprocessing tool that loads jpeg or png images to a GeoPackage.

## Features

## Instructions
#####Installation

1. Download Github repository as zip file onto your local hard disk, and unzip it into a folder. 
2. You will find three directories Desktop 10.3.1, 10.4 and 10.4.1. You need to use the directory appropriate to the version of ArcGIS you are using.

#### ArcGIS 10.3.1
1. Copy the contents of the Desktop python directory from the unzipped folder to C:\Python27\ArcGIS10.3\Lib\site-packages.
2. Copy cache2gpkg.py to C:\Python27\ArcGIS10.3
3. Copy gdal18.dll to C:\Program Files (x86)\ArcGIS\Desktop10.3\bin  (note: You may need administrator access)
4. Copy gpkg_scheme.xml to C:\Program Files(x86)\ArcGIS\Desktop10.3\TilingSchemes (note: you may need administrator access)
5. Copy the GeoPackageToolbox.tbx and raster2gpkg.py to a folder. A folder that will not get overwritten.
6. Start ArcGIS Desktop 10.4 and launch ArcToolBox.  Right-click inside ArcToolBox, select “Add Toolbox”, and add the GeoPackage Toolbox from the folder from number 5. . 

#### ArcGIS 10.4
1. Copy the contents of the Desktop python directory from the unzipped folder to C:\Python27\ArcGIS10.4\Lib\site-packages.
2. Copy cache2gpkg.py to C:\Python27\ArcGIS10.4
3. Copy gdal18.dll, GPCoreFunctions.dll, GpRasterFunctions.dll and RasterRenderer.dll to C:\Program Files (x86)\ArcGIS\Desktop10.4\bin  (note: You may need administrator access).
4. Copy gpkg_scheme.xml to C:\Program Files(x86)\ArcGIS\Desktop10.4\TilingSchemes (note: you may need administrator access)
5. Copy the GeoPackageToolbox.tbx and raster2gpkg.py to a folder. A folder that will not get overwritten.
6. Start ArcGIS Desktop 10.4 and launch ArcToolBox.  Right-click inside ArcToolBox, select “Add Toolbox”, and add the GeoPackage Toolbox from the folder from step number 5. 

#### ArcGIS 10.4.1
1. Copy the contents of the Desktop python directory from the unzipped folder to C:\Python27\ArcGIS10.4\Lib\site-packages.
2. Copy cache2gpkg.py and cache2gpkgWGS84.py to C:\Python27\ArcGIS10.4
3. Copy *.dll to C:\Program Files (x86)\ArcGIS\Desktop10.4\bin (note: You may need administrator access).
4. Copy gpkg_scheme.xml and gpkg_schemeWGS84.xml to C:\Program Files(x86)\ArcGIS\Desktop10.4\TilingSchemes (note: you may need administrator access)
5. Copy the GeoPackageToolbox.tbx , raster2gpkg.py and raster2gpkgWGS84.py to a folder. A folder that will not get overwritten.
6. Start ArcGIS Desktop 10.4.1 and launch ArcToolBox. Right-click inside ArcToolBox, select “Add Toolbox”, and add the GeoPackage Toolbox from the folder from step number 5.

NOTE: Please confirm the Raster To GeoPackage WGS84 Script Location inside the GeoPackage Toolbox before running.


#####To Create a Raster OGC GeoPackage in ArcGIS 
1. In ArcToolBox open the GeoPackage Toolbox and create a new empty OGC GeoPackage using the Create SQLite Database GP tool. Make sure to select "GEOPACKAGE" as the spatial type.
2. In ArcToolBox click on the Raster to GeoPackage tool.  Navigate to the new empty OGC GeoPackage that you just created and select a jpeg or png image to be loaded into the GPKG (e.g. “q0513ne.jpg”). Click OK.

#####To Load a Raster OGC GeoPackage into ArcGIS
1. In ArcToolbox go to Data Management > Layers and Table Views > Make Raster Layer 
2. Navigate to an OGC GeoPackage containing raster data and select the tiles table (e.g. “main.q0513nejpg_tiles”)
3. Accept the default raster layer name.
4. Click OK.
 
#####Data
example.gpkg - Sample empty geopackage. 

q0513ne.jpg - Sample image for loading.

## Requirements

* ArcGIS Desktop 10.3 with http://support.esri.com/en/downloads/patches-servicepacks/view/productid/66/metaid/2200
* ArcGIS Desktop 10.3.1
* ArcGIS Desktop 10.4
* ArcGIS Desktop 10.4.1

## Resources
* [ArcGIS Blog](http://blogs.esri.com/esri/arcgis/)

## Issues

Find a bug or want to request a new feature?  Please let us know by submitting an issue.

## Contributing

Esri welcomes contributions from anyone and everyone. Please see our [guidelines for contributing](https://github.com/esri/contributing).

## Licensing
Copyright 2016 Esri

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

A copy of the license is available in the repository's [License.txt](License.txt)

[](Esri Tags: ArcGIS Raster GeoPackage GPKG)
[](Esri Language: Python)​
