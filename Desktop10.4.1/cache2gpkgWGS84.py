#!/usr/bin/env python
"""Simple script to package cache folder into GeoPackage.

 Includes GlobalGeodetic class from gdal2tiles.py by Klokan Petr Pridal, klokan at klokan dot cz
 licensed under MIT.

"""
__author__ = 'Robin Princeley'
__copyright__ = "Copyright 2015, Esri"
__license__ = "ASL 2.0"
__version__ = "1.1"
__credits__ = ["Augustus Francis"]
# $Id$

import os
import sys
import re
import math
import sqlite3
import fnmatch
import collections
try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET


TILES_TABLE_SUFFIX = '_tiles'             # Added to basename to create table_name
TILES_TABLE_PREFIX = 'table_'             # Used if basename starts with a non alphabet character

class GlobalGeodetic(object):
    """

    GlobalGeodetic class is licensed under MIT:
	Copyright (c) 2008, Klokan Petr Pridal

		Permission is hereby granted, free of charge, to any person obtaining a
		copy of this software and associated documentation files (the "Software"),
		to deal in the Software without restriction, including without limitation
		the rights to use, copy, modify, merge, publish, distribute, sublicense,
		and/or sell copies of the Software, and to permit persons to whom the
		Software is furnished to do so, subject to the following conditions:

		The above copyright notice and this permission notice shall be included
		in all copies or substantial portions of the Software.

		THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
		OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
		FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
		THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
		LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
		FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
		DEALINGS IN THE SOFTWARE.

    TMS Global Geodetic Profile
    ---------------------------

    Functions necessary for generation of global tiles in Plate Carre projection,
    EPSG:4326, "unprojected profile".

    Such tiles are compatible with Google Earth (as any other EPSG:4326 rasters)
    and you can overlay the tiles on top of OpenLayers base map.
    
    Pixel and tile coordinates are in TMS notation (origin [0,0] in bottom-left).

    What coordinate conversions do we need for TMS Global Geodetic tiles?

      Global Geodetic tiles are using geodetic coordinates (latitude,longitude)
      directly as planar coordinates XY (it is also called Unprojected or Plate
      Carre). We need only scaling to pixel pyramid and cutting to tiles.
      Pyramid has on top level two tiles, so it is not square but rectangle.
      Area [-180,-90,180,90] is scaled to 512x256 pixels.
      TMS has coordinate origin (for pixels and tiles) in bottom-left corner.
      Rasters are in EPSG:4326 and therefore are compatible with Google Earth.

         LatLon      <->      Pixels      <->     Tiles     

     WGS84 coordinates   Pixels in pyramid  Tiles in pyramid
         lat/lon         XY pixels Z zoom      XYZ from TMS 
        EPSG:4326                                           
         .----.                ----                         
        /      \     <->    /--------/    <->      TMS      
        \      /         /--------------/                   
         -----        /--------------------/                
       WMS, KML    Web Clients, Google Earth  TileMapService
    """

    def __init__(self, tileSize=256):
        self.tileSize = tileSize
        self.resFact = 360.0 / self.tileSize
        self.origin = [-180, -90, 0, 90]

    def LatLonToPixels(self, lat, lon, zoom):
        "Converts lat/lon to pixel coordinates in given zoom of the EPSG:4326 pyramid"

        res = self.resFact / 2**zoom
        px = (180 + lat) / res
        py = (90 + lon) / res
        return px, py

    def PixelsToTile(self, px, py):
        "Returns coordinates of the tile covering region in pixel coordinates"

        tx = int(math.ceil(px / float(self.tileSize)) - 1)
        ty = int(math.ceil(py / float(self.tileSize)) - 1)
        return tx, ty

    def LatLonToTile(self, lat, lon, zoom):
        "Returns the tile for zoom which covers given lat/lon coordinates"

        px, py = self.LatLonToPixels(lat, lon, zoom)
        return self.PixelsToTile(px, py)

    def MatrixDim(self, zoom):
        "Matrix Wdith and Height for given zoom level"

        return 2**zoom

    def Resolution(self, zoom):
        "Resolution (arc/pixel) for given zoom level (measured at Equator)"

        return self.resFact / 2**zoom
        #return 180 / float( 1 << (8+zoom) )

    def ZoomForPixelSize(self, pixelSize):
        "Maximal scaledown zoom of the pyramid closest to the pixelSize."

        for i in range(MAXZOOMLEVEL):
            if pixelSize > self.Resolution(i):
                if i != 0:
                    return i - 1
                else:
                    return 0  # We don't want to scale up

    def TileBounds(self, tx, ty, zoom):
        "Returns bounds of the given tile"
        res = self.resFact / 2**zoom
        return (tx * self.tileSize * res - 180, ty * self.tileSize * res - 90,
                (tx + 1) * self.tileSize * res - 180,
                (ty + 1) * self.tileSize * res - 90)

    def TileLatLonBounds(self, tx, ty, zoom):
        "Returns bounds of the given tile in the SWNE form"
        b = self.TileBounds(tx, ty, zoom)
        return (b[1], b[0], b[3], b[2])

class Point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0

    def __init__(self, x, y):
        self.x = x
        self.y = y

class Cache:
    def __init__(self):
        self.path = None
        self.xmlTree = None
        self.wkt = None
        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.matrix_min_x = None
        self.matrix_min_y = None
        self.matrix_max_x = None
        self.matrix_max_y = None
        self.tileStart = None
        self.tileStop = None
        self.levels = []
        self.geodetic = GlobalGeodetic()
        self.verbose = False
        self.srs_id = 0
        self.srs_org_id = 3857
        self.srs_org_name = 'EPSG'
        self.level_info = collections.namedtuple('level_info', ['startX', 'startY',
                                                                'stopX', 'stopY',
                                                                'matrix_width', 'matrix_height',
                                                                'zoom_level',
                                                                'pixel_x_size', 'pixel_y_size',
                                                                'offset_x', 'offset_y'])
        self.level_infos = []

    def deg2num(self, lat_deg, lon_deg, zoom):
        lat_rad = math.radians(lat_deg)
        n = 2.0 ** zoom
        xtile = int((lon_deg + 180.0) / 360.0 * n)
        ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        return (xtile, ytile)

    def num2deg(self, xtile, ytile, zoom):
        n = 2.0 ** zoom
        lon_deg = xtile / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
        lat_deg = math.degrees(lat_rad)
        return (lat_deg, lon_deg)

    def parseXML(self, path):
        self.xmlTree = ET.ElementTree(file=path)
        wktElement = self.xmlTree.iterfind('SpatialReference/WKT')
        if wktElement is not None:
            self.wkt = next(wktElement).text
        originElement = self.xmlTree.iterfind('XMin')
        if originElement is not None:
            self.min_x = float(next(originElement).text)
        originElement = self.xmlTree.iterfind('YMin')
        if originElement is not None:
            self.min_y = float(next(originElement).text)
        originElement = self.xmlTree.iterfind('XMax')
        if originElement is not None:
            self.max_x = float(next(originElement).text)
        originElement = self.xmlTree.iterfind('YMax')
        if originElement is not None:
            self.max_y = float(next(originElement).text)

        print("self.max_x = {0}".format(self.max_x))
        print("self.max_y = {0}".format(self.max_y))

        if self.max_y > self.min_y:
            tmp = self.min_y
            self.min_y = self.max_y
            self.max_y = tmp
        
        print("self.max_x = {0}".format(self.max_x))
        print("self.max_y = {0}".format(self.max_y))
    
        if self.wkt is None or self.min_x is None or self.min_y is None or self.max_x is None or self.max_y is None:
            return False
        latestWKIDElement = self.xmlTree.iterfind('SpatialReference/LatestWKID')
        if latestWKIDElement is not None:
            self.srs_id = int(next(latestWKIDElement).text)
        ulLatLon = self.min_x, self.max_y
        lrLatLon = self.max_x, self.min_y
        print("Lat/Lon: {0}, {1}, {2}, {3}".format(ulLatLon[0], ulLatLon[1], lrLatLon[0], lrLatLon[1]))
        #startX, startY, stopX, stopY = self.getTileStartStopLL(ulLatLon[0], ulLatLon[1], lrLatLon[0], lrLatLon[1], self.levels[0])
        #ulLatLon = self.num2deg(startX, startY, self.levels[0])
        #lrLatLon = self.num2deg(stopX, stopY, self.levels[0])
        self.matrix_min_x, self.matrix_max_y = ulLatLon[0], ulLatLon[1]
        self.matrix_max_x, self.matrix_min_y = lrLatLon[0], lrLatLon[1]
        for index, level in enumerate(self.levels):
            if index == 0:
                startX, startY, stopX, stopY = self.getTileStartStopL0(self.levels[0])
            else:
                prev = self.level_infos[index - 1]
                startX = prev.startX * 2
                startY = prev.startY * 2
                stopX = prev.stopX * 2
                stopY = prev.stopY * 2
            self.level_infos.append(self.level_info(startX=startX, startY=startY, stopX=stopX, stopY=stopY,
                                                    matrix_width=self.geodetic.MatrixDim(level), 
                                                    matrix_height=self.geodetic.MatrixDim(level),
                                                    zoom_level=level,
                                                    pixel_x_size=self.geodetic.Resolution(level),
                                                    pixel_y_size=self.geodetic.Resolution(level),
                                                    offset_x=0, offset_y=0))
            print("Tile(s)[{0}]: {1}, {2}, {3}, {4}".format(level, startX, startY, stopX-1, stopY-1))
        return True

    def findZ(self, path):
        for entry in os.listdir(path):
            entry_path = os.path.join(path, entry)
            entry_lower = entry.lower()
            if os.path.isdir(entry_path) and fnmatch.fnmatch(entry_lower, 'l??'):
                level_str = entry_lower.split('l')
                if len(level_str) == 2:
                    self.levels.append(int(level_str[1]))
        if not self.levels:
            return False
        self.levels = sorted(self.levels)
        print("Found level(s): {0}".format(str(self.levels).strip('[]')))
        return True

    def getTileStartStopL0(self, level):
        level0Path = os.path.join(self.path, "_alllayers", "L{0:02d}".format(level))
        rows = sorted(os.listdir(level0Path))
        cols = sorted(os.listdir(os.path.join(level0Path, rows[0])))
        startX = int(cols[0][1:-4], 16)
        stopX = int(cols[-1][1:-4], 16)
        startY = int(rows[0][1:], 16)
        stopY = int(rows[-1][1:], 16)
        return startX, startY, stopX, stopY

    def getTileStartStopLL(self, min_x, min_y, max_x, max_y, level):
        if level not in self.levels:
            return (0, 0), (0, 0)
        startX, startY = self.deg2num(min_x, min_y, level)
        stopX, stopY = self.deg2num(max_x, max_y, level)
        return startX, startY, stopX+1, stopY+1

    def getTileStartStop(self, level):
        if level not in self.levels:
            return (0, 0), (0, 0)
        tileStart = self.geodetic.LatLonToTile(self.min_x, self.min_y, level)
        tileStop = self.geodetic.LatLonToTile(self.max_x, self.max_y, level)
        startX = tileStart[0]
        startY = tileStart[1]
        stopX = tileStop[0]
        stopY = tileStop[1]
        return startX, startY, stopX+1, stopY+1

    def getTilePath(self, x, y, level):
        levelPath = "L{0:02d}".format(level)
        rowPath = "R{0:08x}".format(y)
        columnPath = "C{0:08x}".format(x)
        return os.path.join(self.path, "_alllayers", levelPath, rowPath, columnPath)

    def findTile(self, path):
        jpgPath = path + '.jpg'
        if os.path.exists(jpgPath):
            return jpgPath
        jpegPath = path + '.jpeg'
        if os.path.exists(jpegPath):
            return jpegPath
        pngPath = path + '.png'
        if os.path.exists(pngPath):
            return pngPath
        return None

    def checkTiles(self):
        for level in self.levels:
            startX, startY, stopX, stopY = self.getTileStartStop(level)
            for y in range(startY, stopY):
                for x in range(startX, stopX):
                    tilePath = self.getTilePath(x, y, level)
                    foundTilePath = self.findTile(tilePath)
                    if foundTilePath is None:
                        print("Missing tile: {0}.png/.jpg".format(tilePath))
                    elif self.verbose:
                        print("Found tile: {0}".format(tilePath))
        print("Required tiles found at expected locations.")
        return True

    def open(self, path):
        if not os.path.isdir(path):
            return False
        self.path = path
        levelsDir = os.path.join(self.path, '_alllayers')
        if not self.findZ(levelsDir):
            return False
        xmlFile = os.path.join(self.path, 'conf.cdi')
        if not self.parseXML(xmlFile):
            return False
        return True

class GeoPackage:
    """
    Simple class to add tiles to an existing or new GeoPackage using GDAL.
    """

    def __init__(self):
        self.connection = None
        self.filename = None
        self.tile_width = 256
        self.tile_height = 256
        self.sr_organization = "NONE"
        self.sr_organization_coordsys_id = 0
        self.sr_description = None
        self.description = None
        self.cache = Cache()
        self.verbose = False

    def __del__(self):
        if self.connection is not None:
            self.connection.close()

    def write_srs(self, srs_name):
        """
        Write SRS to gpkg_spatial_ref_sys table and return srs_id.
        @param wkt: WKT string.
        @param srs_name: Value for srs_name field.
        @return: srs_id for new entry or -1 (undefined cartesian)
        """
        if self.cache.wkt is None:
            return -1
        result = self.connection.execute("""SELECT * FROM gpkg_spatial_ref_sys WHERE srs_id=?;""",
                                         (self.cache.srs_id,)).fetchone()
        if result is None:
            self.connection.execute(
                """
                INSERT INTO gpkg_spatial_ref_sys(srs_name, srs_id, organization, organization_coordsys_id, definition)
                            VALUES(?, ?, ?, ?, ?)
                """, (srs_name, self.cache.srs_id, self.cache.srs_org_name, self.cache.srs_org_id, self.cache.wkt))
            self.connection.commit()
            return self.cache.srs_id
        else:
            return result['srs_id']

    def add_cache(self, path):
        if not self.cache.open(path):
            return False
        identifier = os.path.basename(path)
        table_name = re.sub('[.~,;-]', '', identifier + TILES_TABLE_SUFFIX)
        if not table_name[0].isalpha():
            table_name = TILES_TABLE_PREFIX + table_name
        #table_name = table_name.lower()
        if self.connection.execute("""SELECT * FROM gpkg_contents WHERE identifier=? OR table_name=?""",
                                   (identifier, table_name)).fetchone() is not None:
            print("An entry with identifier {0} and/or table_name {1} already exists in gpkg_contents.".format(identifier, table_name))
            return False
        if self.cache.srs_id == 3857:
            srs_id = self.write_srs('Web Mercator')
        if self.description is None:
            self.description = path
        try:
            self.connection.execute(
                """
                INSERT INTO gpkg_contents(table_name, data_type, identifier, description, min_x, min_y, max_x, max_y, srs_id)
                VALUES(?, 'tiles', ?, ?, ?, ?, ?, ?, ?);
                """,
                (table_name, identifier, self.description, self.cache.matrix_min_x, self.cache.matrix_min_y, self.cache.matrix_max_x, self.cache.matrix_max_y, self.cache.srs_id)
            )
            self.connection.execute(
                """
                INSERT INTO gpkg_tile_matrix_set(table_name, srs_id, min_x, min_y, max_x, max_y)
                VALUES(?, ?, ?, ?, ?, ?);
                """,
                (table_name, self.cache.srs_id, self.cache.geodetic.origin[0], self.cache.geodetic.origin[1], 
                self.cache.geodetic.origin[2], self.cache.geodetic.origin[3])
            )
            sql_string = """
                CREATE TABLE """ + table_name + """ (
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  zoom_level INTEGER NOT NULL,
                  tile_column INTEGER NOT NULL,
                  tile_row INTEGER NOT NULL,
                  tile_data BLOB NOT NULL,
                  UNIQUE (zoom_level, tile_column, tile_row) );
                """
            self.connection.execute(sql_string)
            sql_string = """
                CREATE TRIGGER '""" + table_name + """_zoom_insert'
                BEFORE INSERT ON '""" + table_name + """'
                FOR EACH ROW BEGIN
                SELECT RAISE(ABORT, 'insert on table """ + table_name + """ violates constraint: zoom_level not specified for table in gpkg_tile_matrix')
                WHERE NOT (NEW.zoom_level IN (SELECT zoom_level FROM gpkg_tile_matrix WHERE table_name = '""" + table_name + """')) ;
                END
            """
            self.connection.execute(sql_string)
            sql_string = """
                CREATE TRIGGER '""" + table_name + """_zoom_update'
                BEFORE UPDATE OF zoom_level ON '""" + table_name + """'
                FOR EACH ROW BEGIN
                SELECT RAISE(ABORT, 'update on table """ + table_name + """ violates constraint: zoom_level not specified for table in gpkg_tile_matrix')
                WHERE NOT (NEW.zoom_level IN (SELECT zoom_level FROM gpkg_tile_matrix WHERE table_name = '""" + table_name + """')) ;
                END
            """
            self.connection.execute(sql_string)
            sql_string = """
                CREATE TRIGGER '""" + table_name + """_tile_column_insert'
                BEFORE INSERT ON '""" + table_name + """'
                FOR EACH ROW BEGIN
                SELECT RAISE(ABORT, 'insert on table """ + table_name + """ violates constraint: tile_column cannot be < 0')
                WHERE (NEW.tile_column < 0) ;
                SELECT RAISE(ABORT, 'insert on table """ + table_name + """ violates constraint: tile_column must by < matrix_width specified for table and zoom level in gpkg_tile_matrix')
                WHERE NOT (NEW.tile_column < (SELECT matrix_width FROM gpkg_tile_matrix WHERE table_name = '""" + table_name + """' AND zoom_level = NEW.zoom_level));
                END
            """
            self.connection.execute(sql_string)
            sql_string = """
                CREATE TRIGGER '""" + table_name + """_tile_column_update'
                BEFORE UPDATE OF tile_column ON '""" + table_name + """'
                FOR EACH ROW BEGIN
                SELECT RAISE(ABORT, 'update on table """ + table_name + """ violates constraint: tile_column cannot be < 0')
                WHERE (NEW.tile_column < 0) ;
                SELECT RAISE(ABORT, 'update on table """ + table_name + """ violates constraint: tile_column must by < matrix_width specified for table and zoom level in gpkg_tile_matrix')
                WHERE NOT (NEW.tile_column < (SELECT matrix_width FROM gpkg_tile_matrix WHERE table_name = '""" + table_name + """' AND zoom_level = NEW.zoom_level));
                END
            """
            self.connection.execute(sql_string)
            sql_string = """
                CREATE TRIGGER '""" + table_name + """_tile_row_insert'
                BEFORE INSERT ON '""" + table_name + """'
                FOR EACH ROW BEGIN
                SELECT RAISE(ABORT, 'insert on table """ + table_name + """ violates constraint: tile_row cannot be < 0')
                WHERE (NEW.tile_row < 0) ;
                SELECT RAISE(ABORT, 'insert on table """ + table_name + """ violates constraint: tile_row must by < matrix_height specified for table and zoom level in gpkg_tile_matrix')
                WHERE NOT (NEW.tile_row < (SELECT matrix_height FROM gpkg_tile_matrix WHERE table_name = '""" + table_name + """' AND zoom_level = NEW.zoom_level));
                END
            """
            self.connection.execute(sql_string)
            sql_string = """
                CREATE TRIGGER '""" + table_name + """_tile_row_update'
                BEFORE UPDATE OF tile_row ON '""" + table_name + """'
                FOR EACH ROW BEGIN
                SELECT RAISE(ABORT, 'update on table """ + table_name + """ violates constraint: tile_row cannot be < 0')
                WHERE (NEW.tile_row < 0) ;
                SELECT RAISE(ABORT, 'update on table """ + table_name + """ violates constraint: tile_row must by < matrix_height specified for table and zoom level in gpkg_tile_matrix')
                WHERE NOT (NEW.tile_row < (SELECT matrix_height FROM gpkg_tile_matrix WHERE table_name = '""" + table_name + """' AND zoom_level = NEW.zoom_level));
                END
            """
            self.connection.execute(sql_string)
        except sqlite3.Error as e:
            print("Error inserting entries into gpkg_contents and/or other tables: {0}".format(e.args[0]))
            return False
        self.connection.commit()
        for level in self.cache.level_infos:
            try:
                self.connection.execute(
                    """
                    INSERT INTO gpkg_tile_matrix(table_name, zoom_level, matrix_width, matrix_height, tile_width,
                                                 tile_height, pixel_x_size, pixel_y_size)
                    VALUES(?, ?, ?, ?, ?, ?, ?, ?);
                    """,
                    (table_name, level.zoom_level, level.matrix_width, level.matrix_height,
                     self.tile_width, self.tile_height, level.pixel_x_size, level.pixel_y_size)
                )
            except sqlite3.Error as e:
                print("Error inserting entry into gpkg_tile_matrix for overview {0}: {1}".format(level.zoom_level, e.args[0]))
                return False
            if not self.write_level(table_name, level):
                print("Error writing full resolution tiles.")
                return False
        self.connection.commit()
        return True

    def write_level(self, table_name, level):
        """
        Write one zoom/resolution level into pyramid data table.
        @param table_name: Name of table to write pyramid data into.
        @param zoom_level: Zoom/Resolution level to write.
        @return: True on success, False on failure.
        """
        for tile_row in range(level.startY, level.stopY):
            for tile_column in range(level.startX, level.stopX):
                tilePath = self.cache.getTilePath(tile_column, tile_row, level.zoom_level)
                foundTilePath = self.cache.findTile(tilePath)
                if foundTilePath is None:
                    if self.verbose:
                        print("{0}[.jpg/.png] not found, skipping.".format(tilePath))
                else:

                    if not self.write_tile(foundTilePath, table_name, level.zoom_level,
                                           (tile_row ) + level.offset_y,
                                           (tile_column ) + level.offset_x):
                        print("Error writing image tiles for level {0} to database.".format(level.zoom_level))
                        return False
        return True

    def write_tile(self, filename, table_name, zoom_level, tile_row, tile_column):
        """
        Extract specified tile from source dataset and write as a blob into GeoPackage, expanding colormap if required.
        @param table_name: Name of table to write pyramid data into.
        @param zoom_level: Zoom/Resolution level to write.
        @param tile_row: Tile index (Y).
        @param tile_column: Tile index (X).
        @return: True on success, False on failure.
        """
        size = os.stat(filename).st_size
        if size == 0:
            print("Tile {0} is 0 bytes, ignoring.".format(filename))
            return True
        try:
            in_file = open(filename, 'rb')
            tile_data = in_file.read(size)
            in_file.close()
        except IOError as e:
            print("Error reading tile {0} : {1}".format(filename, e.args[0]))
            return False
        try:
            self.connection.execute(
                """
                INSERT INTO """ + table_name + """(zoom_level, tile_column, tile_row, tile_data)
                VALUES (?, ?, ?, ?);
                """,
                (zoom_level, tile_column, tile_row, buffer(tile_data))
            )
        except sqlite3.Error as e:
            print("Error inserting blob for tile {0}, {1}: {2}".format(tile_column, tile_row, e.args[0]))
            return False
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
            print("ERROR: SQLite error while creating core tables and triggers: {0}".format(e.args[0]))
            return False
        return True


def usage():
    print("Usage: gdal2gpkg [-sr_org organization] [-sr_sysid identifier] [-sr_desc description]\n"
          "                 [-v] path gpkgname")
    return 2


def equal(a, b):
    """
    Case insensitive string compare.
    @param a: String to compare.
    @param b: String to compare.
    @return: True if equal, False if not.
    """
    return a.lower() == b.lower()


def cache2gpkg(cache_path, gpkg_filename, verbose=False, **kwargs):
    gpkg = GeoPackage()
    gpkg.verbose = gpkg.cache.verbose = verbose
    if kwargs is not None:
        for key, value in kwargs.iteritems():
            if equal(key, "sr_org"):
                gpkg.sr_organization = value
            elif equal(key, "sr_sysid"):
                gpkg.sr_organization_coordsys_id = value
            elif equal(key, "-sr_desc"):
                gpkg.sr_description = value
    if cache_path is None or gpkg_filename is None:
        print("ERROR: Failed to open or create {0}".format(gpkg_filename))
        return False
    if not gpkg.open(gpkg_filename):
        print("ERROR: Failed to open or create {0}".format(gpkg_filename))
        return False
    if not gpkg.add_cache(cache_path):
        print("ERROR: Adding {0} to {1} failed".format(cache_path, gpkg_filename))
        return False
    gpkg = None
    return True
