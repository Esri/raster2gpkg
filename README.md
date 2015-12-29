#raster2gpkg

This is a Python based Geoprocessing tool that loads jpeg or png images to a GeoPackage. 

## Features

## Instructions
Installation
1. Download Github repository as zip file onto your local hard disk, and unzip it into a folder. 
2. Copy the contents of the python directory from the unzipped folder to C:\Python27\ArcGIS10.3\Lib\site-packages.
3. Copy cache2gpkg.py to C:\Python27\ArcGIS10.3
4. Copy gdal18.dll to C:\Program Files (x86)\ArcGIS\Desktop10.3\bin  (note: you may need administrator access)
5. Copy gpkg_scheme.xml to C:\Program Files(x86)\ArcGIS\Desktop10.3\TilingSchemes (note: you may need administrator access)
6. Start ArcGIS Desktop 10.3.1 and launch ArcToolBox.  Right-click inside ArcToolBox, select “Add Toolbox”, and add the GeoPackage Toolbox from your unzipped folder. 
To Create a Raster OGC GeoPackage in ArcGIS 
1. In ArcToolBox open the GeoPackage Toolbox and create a new empty OGC GeoPackage using the Create SQLite Database tool. Make sure to select "GEOPACKAGE" as the spatial type.
2. In ArcToolBox click on the Raster to GeoPackage tool.  Navigate to the new empty OGC GeoPackage that you just created and select a jpeg or png image to be loaded into the GPKG (e.g. “q0513ne.jpg”). Click OK.

To Load a Raster OGC GeoPackage into ArcGIS
1. In ArcToolbox go to Data Management > Layers and Table Views > Make Raster Layer 
2. Navigate to an OGC GeoPackage containing raster data and select the tiles table (e.g. “main.q0513nejpg_tiles”)
3. Accept the default raster layer name.
4. Click OK. lick OK. The python script, "raster2gpkg.py is located in \\jotunheimen\C\gdal2gpkgWorkup. It's also been loaded into the GP tool and is not needed for install.
 
9) Data
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

A copy of the license is available in the repository's [License.txt](License.txt)

[](Esri Tags: ArcGIS Raster GeoPackage GPKG)
[](Esri Language: Python)​
