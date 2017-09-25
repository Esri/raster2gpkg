#!/usr/bin/env python
#------------------------------------------------------------------------------
# Copyright 2015 Esri
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#------------------------------------------------------------------------------

# ==================================================
# raster2gpkg.py
# --------------------------------------------------
# author: Esri
# company: Esri
# ==================================================
# description: Simple script to add a dataset into an existing or new GeoPackage.
# Based on other python utilities but made PEP compliant.
# ==================================================
# ==========================

# $Id$

import arcpy
import os
import sys
import re
import math
import sqlite3
import numpy
import tempfile
import collections
from osgeo import gdal
from osgeo.gdalconst import *
import cache2gpkgWGS84
import shutil


TILES_TABLE_SUFFIX = '_tiles'             # Added to basename to create table_name
TILES_TABLE_PREFIX = 'table_'             # Used if basename starts with a non alphabet character

class GeoPackage:
    """
    Simple class to add tiles to an existing or new GeoPackage using GDAL.
    """

    def __init__(self, fmt='image/jpeg'):
        self.connection = None
        self.filename = None
        self.tile_width = 256
        self.tile_height = 256
        self.full_width = 0
        self.full_height = 0
        self.format = fmt
        self.format_options = list()
        self.sr_organization = "NONE"
        self.sr_organization_coordsys_id = 0
        self.sr_description = None
        self.data_type = GDT_Unknown
        self.mem_driver = gdal.GetDriverByName("MEM")
        if self.format == 'image/jpeg':
            self.driver = gdal.GetDriverByName('JPEG')
        elif self.format == 'image/png':
            self.driver = gdal.GetDriverByName("PNG")
        if self.driver is None or self.mem_driver is None:
            raise RuntimeError("Can't find appropriate GDAL driver for MEM and/or format ", self.format)
        self.jpeg_options = []
        self.ResolutionLayerInfo = collections.namedtuple('ResolutionLayerInfo', ['factor_x', 'factor_y',
                                                                                  'pixel_x_size', 'pixel_y_size',
                                                                                  'width', 'height',
                                                                                  'matrix_width', 'matrix_height',
                                                                                  'expected_pixel_x_size',
                                                                                  'expected_pixel_y_size'])
        self.overviews = []
        self.tile_lod_info = [
			[0.703125, 295828763.79585470937713011037],
			[0.3515625, 147914381.89792735468856505518],
			[0.17578125, 73957190.948963677344282527592],
			[0.087890625, 36978595.474481838672141263796],
			[0.0439453125, 18489297.737240919336070631898],
			[0.02197265625, 9244648.868620459668035315949],
			[0.010986328125, 4622324.4343102298340176579745],
			[0.0054931640625, 2311162.2171551149170088289872],
			[0.00274658203125, 1155581.1085775574585044144937],
			[0.001373291015625, 577790.55428877872925220724681],
			[0.0006866455078125, 288895.27714438936462610362340],
			[0.00034332275390625, 144447.63857219468231305181171],
			[0.000171661376953125, 72223.819286097341156525905853],
			[0.0000858306884765625, 36111.909643048670578262952926],
			[0.00004291534423828125, 18055.954821524335289131476463],
			[0.000021457672119140625, 9027.977410762167644565738231],
			[0.0000107288360595703125, 4513.9887053810838222828691158],
			[0.00000536441802978515625, 2256.9943526905419111414345579],
			[0.000002682209014892578125, 1128.4971763452709555707172788],
			[0.0000013411045074462890625, 564.24858817263547778535863938],
			[0.00000067055225372314453125, 282.12429408631773889267931988],
			[0.000000335276126861572265625, 141.06214704315886944633965975],
			[0.0000001676380634307861328125, 70.531073521579434723169829875],
            ]

    def __del__(self):
        if self.connection is not None:
            self.connection.close()

    def add_dataset(self, filename):
        raster_desc = arcpy.Describe(filename)
        if raster_desc is None:
            arcpy.AddError("Failed to describe input")
            return False

        arcpy.AddMessage("Raster described {0}".format(filename))

        srs = raster_desc.spatialReference
        extent = raster_desc.Extent

        #compute the new projected extent
        new_srs = arcpy.SpatialReference(4326)
        new_extent = extent.projectAs(new_srs)

        #compute the new projected source cellsize using the extent of one cell from the input
        #extent of one cell in the input is:
        #xmin = input_xmin
        #xmax = input_xmin + input_x_cell_size
        #ymin = input_ymin
        #ymax = input_ymin + input_y_cell_size

        input_cs_x = float(str(arcpy.GetRasterProperties_management(filename, 'CELLSIZEX')))
        input_cs_y = float(str(arcpy.GetRasterProperties_management(filename, 'CELLSIZEY')))

        arcpy.AddMessage("Input CS X: {0}".format(input_cs_x))
        arcpy.AddMessage("Input CS Y: {0}".format(input_cs_y))

        #update the 'extent' with cell extent
        extent.XMax = extent.XMin + input_cs_x
        extent.YMax = extent.YMin + input_cs_y

        new_cell_extent = extent.projectAs(new_srs)

        # Get the cell size of the projected_raster
        pixel_x_size = new_cell_extent.width
        pixel_y_size = new_cell_extent.height

        # Get the extent of the projected_raster
        max_y = new_extent.YMax
        min_y = new_extent.YMin
        min_x = new_extent.XMin
        max_x = new_extent.XMax

        if pixel_x_size == 0 or pixel_y_size == 0:
            print("Invalid pixel sizes")
            return False

        if min_x == 0.0 or min_y == 0.0 or max_x == 0.0 or max_y == 0.0:
            print("Invalid extent")
            return False

        # Set the source_pixel_size to twice the original resolution to compute the max scale
        source_pixel_size = pixel_x_size

        # Set the max cell size to twice the cell size required for a super tile size of 512
        # This seems to be inaccurate
        # max_pixel_size = (max_x - min_x) / 256

        new_dims = (max_x - min_x) // source_pixel_size + ((max_x - min_x) % source_pixel_size > 0)
        zoom_levels = int(math.log(new_dims/512.0, 2) + 1) if new_dims >= 512 else 0

        #Prefering SubScaling
        for counter, lod_info in enumerate(self.tile_lod_info):
            print(lod_info[0], lod_info[1])
            if source_pixel_size > lod_info[0]:
                break
            max_scale = lod_info[1]

        min_scale = (self.tile_lod_info[counter - 1 - zoom_levels][1] 
                     if counter -1 - zoom_levels > 0 else self.tile_lod_info[0][1])

        tempFolder = tempfile.mkdtemp(suffix='_gpkg_cache')
        cacheName = os.path.basename(filename)
        
        arcmap_dir = arcpy.GetInstallInfo()['InstallDir']
        if (arcpy.Exists(arcmap_dir) == False): 
            raise arcpy.ExecuteError("Arc Installation not found.")
        arcmap_tilingscheme_dir = os.path.join(arcmap_dir, 'TilingSchemes', 'gpkg_schemeWGS84.xml')

        if os.path.isfile(arcmap_tilingscheme_dir) == False:
            raise arcpy.ExecuteError("Tiling Scheme File is Missing.")

        arcpy.AddMessage("Generating tiles in {0}".format(tempFolder))

        arcpy.ManageTileCache_management(in_cache_location=tempFolder,
                                         manage_mode='RECREATE_ALL_TILES',
                                         in_cache_name=cacheName,
                                         in_datasource=filename,
                                         tiling_scheme='IMPORT_SCHEME',
                                         import_tiling_scheme=arcmap_tilingscheme_dir)

        arcpy.AddMessage("Creating GeoPackage {0}".format(self.filename))

        cachePath = tempFolder + "/" + cacheName
        cache2gpkgWGS84.cache2gpkg(cachePath, self.filename, True)

        arcpy.AddMessage("GeoPackage {0} created".format(self.filename))

        identifier = os.path.basename(cachePath)
        table_name = re.sub('[.~,;-]', '', identifier + TILES_TABLE_SUFFIX)
        if not table_name[0].isalpha():
            table_name = TILES_TABLE_PREFIX + table_name

        geopackage_ds = gdal.Open(self.filename + "\\" + table_name)
        geopackage_ds.SetMetadataItem("ZOOM_LEVEL_STRATEGY", "UPPER")

        ### Cleanup
        new_srs = None
        new_extent = None
        raster_desc = None
        shutil.rmtree(tempFolder)

        return True

    def open(self, filename):
        """
        Create or open a GeoPackage and create necessary tables and triggers.
        @param filename: Name of sqlite3 database.
        @return: True on success, False on failure.
        """
        self.filename = filename
        try:
            self.connection = sqlite3.connect(filename)
        except sqlite3.Error as e:
            print("Error opening ", filename, ": ", e.args[0])
            return False
        self.connection.row_factory = sqlite3.Row
        try:
            self.connection.execute(
                """
                PRAGMA application_id = 1196437808;
                """
            )
            self.connection.execute(
                """
                CREATE TABLE IF NOT EXISTS gpkg_spatial_ref_sys ( \
                             srs_name TEXT NOT NULL, \
                             srs_id INTEGER NOT NULL PRIMARY KEY, \
                             organization TEXT NOT NULL, \
                             organization_coordsys_id INTEGER NOT NULL, \
                             definition  TEXT NOT NULL, \
                             description TEXT );
                """
            )
            self.connection.execute(
                """
                INSERT INTO gpkg_spatial_ref_sys(srs_name,srs_id,organization,organization_coordsys_id,definition)
                SELECT 'Undefined Cartesian', -1, 'NONE', -1, 'undefined'
                WHERE NOT EXISTS(SELECT 1 FROM gpkg_spatial_ref_sys WHERE srs_id=-1);
                """
            )
            self.connection.execute(
                """
                INSERT INTO gpkg_spatial_ref_sys(srs_name,srs_id,organization,organization_coordsys_id,definition)
                SELECT 'Undefined Geographic', 0, 'NONE', 0, 'undefined'
                WHERE NOT EXISTS(SELECT 1 FROM gpkg_spatial_ref_sys WHERE srs_id=0);
                """
            )
            self.connection.execute(
                """
                INSERT INTO gpkg_spatial_ref_sys(srs_name,srs_id,organization,organization_coordsys_id,definition)
                SELECT 'WGS84', 4326, 'EPSG', 4326, 'GEOGCS["WGS 84",
                                                        DATUM["WGS_1984",
                                                            SPHEROID["WGS 84",6378137,298.257223563,
                                                                AUTHORITY["EPSG","7030"]],
                                                            AUTHORITY["EPSG","6326"]],
                                                        PRIMEM["Greenwich",0,
                                                            AUTHORITY["EPSG","8901"]],
                                                        UNIT["degree",0.01745329251994328,
                                                            AUTHORITY["EPSG","9122"]],
                                                        AUTHORITY["EPSG","4326"]]'
                WHERE NOT EXISTS(SELECT 1 FROM gpkg_spatial_ref_sys WHERE srs_id=4326);
                """
            )
            self.connection.execute(
                """
                    CREATE TABLE IF NOT EXISTS gpkg_contents (
                                 table_name TEXT NOT NULL PRIMARY KEY, \
                                 data_type TEXT NOT NULL, \
                                 identifier TEXT UNIQUE, \
                                 description TEXT DEFAULT '', \
                                 last_change DATETIME NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ',CURRENT_TIMESTAMP)), \
                                 min_x DOUBLE, \
                                 min_y DOUBLE, \
                                 max_x DOUBLE, \
                                 max_y DOUBLE, \
                                 srs_id INTEGER, \
                                 CONSTRAINT fk_gc_r_srs_id FOREIGN KEY (srs_id) REFERENCES gpkg_spatial_ref_sys(srs_id) );
                    """
            )
            self.connection.execute(
                """
                    CREATE TABLE IF NOT EXISTS gpkg_tile_matrix (
                                 table_name TEXT NOT NULL,
                                 zoom_level INTEGER NOT NULL,
                                 matrix_width INTEGER NOT NULL,
                                 matrix_height INTEGER NOT NULL,
                                 tile_width INTEGER NOT NULL,
                                 tile_height INTEGER NOT NULL,
                                 pixel_x_size DOUBLE NOT NULL,
                                 pixel_y_size DOUBLE NOT NULL,
                                 CONSTRAINT pk_ttm PRIMARY KEY (table_name, zoom_level),
                                 CONSTRAINT fk_tmm_table_name FOREIGN KEY (table_name) REFERENCES gpkg_contents(table_name) );
                  """
            )
            self.connection.execute(
                """
                    CREATE TRIGGER IF NOT EXISTS 'gpkg_tile_matrix_zoom_level_insert'
                    BEFORE INSERT ON 'gpkg_tile_matrix'
                    FOR EACH ROW BEGIN
                    SELECT RAISE(ABORT, 'insert on table ''gpkg_tile_matrix'' violates constraint: zoom_level cannot be less than 0')
                    WHERE (NEW.zoom_level < 0);
                    END
                    """
            )
            self.connection.execute(
                """
                    CREATE TRIGGER IF NOT EXISTS 'gpkg_tile_matrix_zoom_level_update'
                    BEFORE UPDATE OF zoom_level ON 'gpkg_tile_matrix'
                    FOR EACH ROW BEGIN
                    SELECT RAISE(ABORT, 'update on table ''gpkg_tile_matrix'' violates constraint: zoom_level cannot be less than 0')
                    WHERE (NEW.zoom_level < 0);
                    END
                    """
            )
            self.connection.execute(
                """
                    CREATE TRIGGER IF NOT EXISTS 'gpkg_tile_matrix_matrix_width_insert'
                    BEFORE INSERT ON 'gpkg_tile_matrix'
                    FOR EACH ROW BEGIN
                    SELECT RAISE(ABORT, 'insert on table ''gpkg_tile_matrix'' violates constraint: matrix_width cannot be less than 1')
                    WHERE (NEW.matrix_width < 1);
                    END
                    """
            )
            self.connection.execute(
                """
                    CREATE TRIGGER IF NOT EXISTS 'gpkg_tile_matrix_matrix_width_update'
                    BEFORE UPDATE OF matrix_width ON 'gpkg_tile_matrix'
                    FOR EACH ROW BEGIN
                    SELECT RAISE(ABORT, 'update on table ''gpkg_tile_matrix'' violates constraint: matrix_width cannot be less than 1')
                    WHERE (NEW.matrix_width < 1);
                    END
                    """
            )
            self.connection.execute(
                """
                    CREATE TRIGGER IF NOT EXISTS 'gpkg_tile_matrix_matrix_height_insert'
                    BEFORE INSERT ON 'gpkg_tile_matrix'
                    FOR EACH ROW BEGIN
                    SELECT RAISE(ABORT, 'insert on table ''gpkg_tile_matrix'' violates constraint: matrix_height cannot be less than 1')
                    WHERE (NEW.matrix_height < 1);
                    END
                    """
            )
            self.connection.execute(
                """
                    CREATE TRIGGER IF NOT EXISTS 'gpkg_tile_matrix_matrix_height_update'
                    BEFORE UPDATE OF matrix_height ON 'gpkg_tile_matrix'
                    FOR EACH ROW BEGIN
                    SELECT RAISE(ABORT, 'update on table ''gpkg_tile_matrix'' violates constraint: matrix_height cannot be less than 1')
                    WHERE (NEW.matrix_height < 1);
                    END
                    """
            )
            self.connection.execute(
                """
                    CREATE TRIGGER IF NOT EXISTS 'gpkg_tile_matrix_pixel_x_size_insert'
                    BEFORE INSERT ON 'gpkg_tile_matrix'
                    FOR EACH ROW BEGIN
                    SELECT RAISE(ABORT, 'insert on table ''gpkg_tile_matrix'' violates constraint: pixel_x_size must be greater than 0')
                    WHERE NOT (NEW.pixel_x_size > 0);
                    END
                    """
            )
            self.connection.execute(
                """
                    CREATE TRIGGER IF NOT EXISTS 'gpkg_tile_matrix_pixel_x_size_update'
                    BEFORE UPDATE OF pixel_x_size ON 'gpkg_tile_matrix'
                    FOR EACH ROW BEGIN
                    SELECT RAISE(ABORT, 'update on table ''gpkg_tile_matrix'' violates constraint: pixel_x_size must be greater than 0')
                    WHERE NOT (NEW.pixel_x_size > 0);
                    END
                    """
            )
            self.connection.execute(
                """
                    CREATE TRIGGER IF NOT EXISTS 'gpkg_tile_matrix_pixel_y_size_insert'
                    BEFORE INSERT ON 'gpkg_tile_matrix'
                    FOR EACH ROW BEGIN
                    SELECT RAISE(ABORT, 'insert on table ''gpkg_tile_matrix'' violates constraint: pixel_y_size must be greater than 0')
                    WHERE NOT (NEW.pixel_y_size > 0);
                    END
                    """
            )
            self.connection.execute(
                """
                    CREATE TRIGGER IF NOT EXISTS 'gpkg_tile_matrix_pixel_y_size_update'
                    BEFORE UPDATE OF pixel_y_size ON 'gpkg_tile_matrix'
                    FOR EACH ROW BEGIN
                    SELECT RAISE(ABORT, 'update on table ''gpkg_tile_matrix'' violates constraint: pixel_y_size must be greater than 0')
                    WHERE NOT (NEW.pixel_y_size > 0);
                    END
                    """
            )
            self.connection.execute(
                """
                    CREATE TABLE IF NOT EXISTS gpkg_tile_matrix_set (
                                 table_name TEXT NOT NULL PRIMARY KEY,
                                 srs_id INTEGER NOT NULL,
                                 min_x DOUBLE NOT NULL,
                                 min_y DOUBLE NOT NULL,
                                 max_x DOUBLE NOT NULL,
                                 max_y DOUBLE NOT NULL,
                                 CONSTRAINT fk_gtms_table_name FOREIGN KEY (table_name) REFERENCES gpkg_contents(table_name),
                                 CONSTRAINT fk_gtms_srs FOREIGN KEY (srs_id) REFERENCES gpkg_spatial_ref_sys (srs_id) );
                  """
            )
            self.connection.commit()
        except sqlite3.Error as e:
            print("ERROR: SQLite error while creating core tables and triggers: ", e.args[0])
            return False
        return True


def usage():
    print("Usage: gdal2gpkg3 datasetname gpkgname")
    return 2


def equal(a, b):
    """
    Case insensitive string compare.
    @param a: String to compare.
    @param b: String to compare.
    @return: True if equal, False if not.
    """
    return a.lower() == b.lower()


def main(argv=None):
    dataset_filename = None
    gpkg_filename = None
    gpkg = GeoPackage()

    dataset_filename = arcpy.GetParameterAsText(0)
    gpkg_filename = arcpy.GetParameterAsText(1)

    image_extension = os.path.splitext(dataset_filename)[1][1:].strip()

    if image_extension.lower() in ('jpg', 'jpeg'):
        gpkg.format = "image/jpeg"
    elif image_extension.lower() == 'png':
        gpkg.format = "image/png"
    else:
        extension = ''

    if not gpkg.open(gpkg_filename):
        print("ERROR: Failed to open or create ", gpkg_filename)
        return 1
    if not gpkg.add_dataset(dataset_filename):
        print("ERROR: Adding ", dataset_filename, " to ", gpkg_filename, " failed")
        return 1
    gpkg = None
    arcpy.AddMessage('\nDone')
    return 0


if __name__ == '__main__':
    version_num = int(gdal.VersionInfo('VERSION_NUM'))
    if version_num < 1800: # because of GetGeoTransform(can_return_null)
        print('ERROR: Python bindings of GDAL 1.8.0 or later required')
        sys.exit(1)
    sys.exit(main(sys.argv))
