#!/usr/bin/env python
"""Simple script to package cache folder into GeoPackage.

 Includes GlobalMercator class from gdal2tiles.py by Klokan Petr Pridal, klokan at klokan dot cz

"""
__author__ = 'Robin Princeley'
__copyright__ = "Copyright 2015, Esri"
__license__ = "MIT"
__version__ = "1.0"

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

class GlobalMercator(object):
    """
    TMS Global Mercator Profile
    ---------------------------

  Functions necessary for generation of tiles in Spherical Mercator projection,
  EPSG:900913 (EPSG:gOOglE, Google Maps Global Mercator), EPSG:3785, OSGEO:41001.

  Such tiles are compatible with Google Maps, Bing Maps, Yahoo Maps,
  UK Ordnance Survey OpenSpace API, ...
  and you can overlay them on top of base maps of those web mapping applications.

    Pixel and tile coordinates are in TMS notation (origin [0,0] in bottom-left).

    What coordinate conversions do we need for TMS Global Mercator tiles::

         LatLon      <->       Meters      <->     Pixels    <->       Tile

     WGS84 coordinates   Spherical Mercator  Pixels in pyramid  Tiles in pyramid
         lat/lon            XY in metres     XY pixels Z zoom      XYZ from TMS
        EPSG:4326           EPSG:900913
         .----.              ---------               --                TMS
        /      \     <->     |       |     <->     /----/    <->      Google
        \      /             |       |           /--------/          QuadTree
         -----               ---------         /------------/
       KML, public         WebMapService         Web Clients      TileMapService

    What is the coordinate extent of Earth in EPSG:900913?

      [-20037508.342789244, -20037508.342789244, 20037508.342789244, 20037508.342789244]
      Constant 20037508.342789244 comes from the circumference of the Earth in meters,
      which is 40 thousand kilometers, the coordinate origin is in the middle of extent.
      In fact you can calculate the constant as: 2 * math.pi * 6378137 / 2.0
      $ echo 180 85 | gdaltransform -s_srs EPSG:4326 -t_srs EPSG:900913
      Polar areas with abs(latitude) bigger then 85.05112878 are clipped off.

    What are zoom level constants (pixels/meter) for pyramid with EPSG:900913?

      whole region is on top of pyramid (zoom=0) covered by 256x256 pixels tile,
      every lower zoom level resolution is always divided by two
      initialResolution = 20037508.342789244 * 2 / 256 = 156543.03392804062

    What is the difference between TMS and Google Maps/QuadTree tile name convention?

      The tile raster itself is the same (equal extent, projection, pixel size),
      there is just different identification of the same raster tile.
      Tiles in TMS are counted from [0,0] in the bottom-left corner, id is XYZ.
      Google placed the origin [0,0] to the top-left corner, reference is XYZ.
      Microsoft is referencing tiles by a QuadTree name, defined on the website:
      http://msdn2.microsoft.com/en-us/library/bb259689.aspx

    The lat/lon coordinates are using WGS84 datum, yeh?

      Yes, all lat/lon we are mentioning should use WGS84 Geodetic Datum.
      Well, the web clients like Google Maps are projecting those coordinates by
      Spherical Mercator, so in fact lat/lon coordinates on sphere are treated as if
      the were on the WGS84 ellipsoid.

      From MSDN documentation:
      To simplify the calculations, we use the spherical form of projection, not
      the ellipsoidal form. Since the projection is used only for map display,
      and not for displaying numeric coordinates, we don't need the extra precision
      of an ellipsoidal projection. The spherical projection causes approximately
      0.33 percent scale distortion in the Y direction, which is not visually noticable.

    How do I create a raster in EPSG:900913 and convert coordinates with PROJ.4?

      You can use standard GIS tools like gdalwarp, cs2cs or gdaltransform.
      All of the tools supports -t_srs 'epsg:900913'.

      For other GIS programs check the exact definition of the projection:
      More info at http://spatialreference.org/ref/user/google-projection/
      The same projection is degined as EPSG:3785. WKT definition is in the official
      EPSG database.

      Proj4 Text:
        +proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0
        +k=1.0 +units=m +nadgrids=@null +no_defs

      Human readable WKT format of EPGS:900913:
         PROJCS["Google Maps Global Mercator",
             GEOGCS["WGS 84",
                 DATUM["WGS_1984",
                     SPHEROID["WGS 84",6378137,298.257223563,
                         AUTHORITY["EPSG","7030"]],
                     AUTHORITY["EPSG","6326"]],
                 PRIMEM["Greenwich",0],
                 UNIT["degree",0.0174532925199433],
                 AUTHORITY["EPSG","4326"]],
             PROJECTION["Mercator_1SP"],
             PARAMETER["central_meridian",0],
             PARAMETER["scale_factor",1],
             PARAMETER["false_easting",0],
             PARAMETER["false_northing",0],
             UNIT["metre",1,
                 AUTHORITY["EPSG","9001"]]]
    """

    def __init__(self, tileSize=256):
        "Initialize the TMS Global Mercator pyramid"
        self.tileSize = tileSize
        self.initialResolution = 2 * math.pi * 6378137 / self.tileSize
        # 156543.03392804062 for tileSize 256 pixels
        self.originShift = 2 * math.pi * 6378137 / 2.0
        # 20037508.342789244

    def LatLonToMeters(self, lat, lon ):
        "Converts given lat/lon in WGS84 Datum to XY in Spherical Mercator EPSG:900913"

        mx = lon * self.originShift / 180.0
        my = math.log( math.tan((90 + lat) * math.pi / 360.0 )) / (math.pi / 180.0)

        my = my * self.originShift / 180.0
        return mx, my

    def MetersToLatLon(self, mx, my ):
        "Converts XY point from Spherical Mercator EPSG:900913 to lat/lon in WGS84 Datum"

        lon = (mx / self.originShift) * 180.0
        lat = (my / self.originShift) * 180.0

        lat = 180 / math.pi * (2 * math.atan( math.exp( lat * math.pi / 180.0)) - math.pi / 2.0)
        return lat, lon

    def PixelsToMeters(self, px, py, zoom):
        "Converts pixel coordinates in given zoom level of pyramid to EPSG:900913"

        res = self.Resolution( zoom )
        mx = px * res - self.originShift
        my = py * res - self.originShift
        return mx, my

    def MetersToPixels(self, mx, my, zoom):
        "Converts EPSG:900913 to pyramid pixel coordinates in given zoom level"

        res = self.Resolution( zoom )
        px = (mx + self.originShift) / res
        py = (my + self.originShift) / res
        return px, py

    def PixelsToTile(self, px, py):
        "Returns a tile covering region in given pixel coordinates"

        tx = int( math.ceil( px / float(self.tileSize) ) - 1 )
        ty = int( math.ceil( py / float(self.tileSize) ) - 1 )
        return tx, ty

    def PixelsToRaster(self, px, py, zoom):
        "Move the origin of pixel coordinates to top-left corner"

        mapSize = self.tileSize << zoom
        return px, mapSize - py

    def MetersToTile(self, mx, my, zoom):
        "Returns tile for given mercator coordinates"

        px, py = self.MetersToPixels( mx, my, zoom)
        return self.PixelsToTile( px, py)

    def TileBounds(self, tx, ty, zoom):
        "Returns bounds of the given tile in EPSG:900913 coordinates"

        minx, miny = self.PixelsToMeters( tx*self.tileSize, ty*self.tileSize, zoom )
        maxx, maxy = self.PixelsToMeters( (tx+1)*self.tileSize, (ty+1)*self.tileSize, zoom )
        return ( minx, miny, maxx, maxy )

    def TileLatLonBounds(self, tx, ty, zoom ):
        "Returns bounds of the given tile in latutude/longitude using WGS84 datum"

        bounds = self.TileBounds( tx, ty, zoom)
        minLat, minLon = self.MetersToLatLon(bounds[0], bounds[1])
        maxLat, maxLon = self.MetersToLatLon(bounds[2], bounds[3])

        return ( minLat, minLon, maxLat, maxLon )

    def Resolution(self, zoom ):
        "Resolution (meters/pixel) for given zoom level (measured at Equator)"

        # return (2 * math.pi * 6378137) / (self.tileSize * 2**zoom)
        return self.initialResolution / (2**zoom)

    def ZoomForPixelSize(self, pixelSize ):
        "Maximal scaledown zoom of the pyramid closest to the pixelSize."

        for i in range(MAXZOOMLEVEL):
            if pixelSize > self.Resolution(i):
                if i!=0:
                    return i-1
                else:
                    return 0 # We don't want to scale up

    def GoogleTile(self, tx, ty, zoom):
        "Converts TMS tile coordinates to Google Tile coordinates"

        # coordinate origin is moved from bottom-left to top-left corner of the extent
        return tx, (2**zoom - 1) - ty

    def QuadTree(self, tx, ty, zoom ):
        "Converts TMS tile coordinates to Microsoft QuadTree"

        quadKey = ""
        ty = (2**zoom - 1) - ty
        for i in range(zoom, 0, -1):
            digit = 0
            mask = 1 << (i-1)
            if (tx & mask) != 0:
                digit += 1
            if (ty & mask) != 0:
                digit += 2
            quadKey += str(digit)

        return quadKey

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
        self.mercator = GlobalMercator()
        self.verbose = False
        self.srs_id = 0
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
        if self.max_y > self.min_y:
            tmp = self.min_y
            self.min_y = self.max_y
            self.max_y = tmp
        if self.wkt is None or self.min_x is None or self.min_y is None or self.max_x is None or self.max_y is None:
            return False
        latestWKIDElement = self.xmlTree.iterfind('SpatialReference/LatestWKID')
        if latestWKIDElement is not None:
            self.srs_id = int(next(latestWKIDElement).text)
        print("Meters: {0}, {1}, {2}, {3}".format(self.min_x, self.min_y, self.max_x, self.max_y))
        ulLatLon = self.mercator.MetersToLatLon(self.min_x, self.min_y)
        lrLatLon = self.mercator.MetersToLatLon(self.max_x, self.max_y)
        print("Lat/Lon: {0}, {1}, {2}, {3}".format(ulLatLon[0], ulLatLon[1], lrLatLon[0], lrLatLon[1]))
        startX, startY, stopX, stopY = self.getTileStartStopLL(ulLatLon[0], ulLatLon[1], lrLatLon[0], lrLatLon[1], self.levels[0])
        ulLatLon = self.num2deg(startX, startY, self.levels[0])
        lrLatLon = self.num2deg(stopX, stopY, self.levels[0])
        self.matrix_min_x, self.matrix_min_y = self.mercator.LatLonToMeters(ulLatLon[0], ulLatLon[1])
        self.matrix_max_x, self.matrix_max_y = self.mercator.LatLonToMeters(lrLatLon[0], lrLatLon[1])
        for index, level in enumerate(self.levels):
            if index == 0:
                startX, startY, stopX, stopY = self.getTileStartStop(self.levels[0])
                matrix_width = stopX - startX
                matrix_height = stopY - startY
            else:
                prev = self.level_infos[index - 1]
                matrix_width = prev.matrix_width * 2
                matrix_height = prev.matrix_height * 2
                startX = prev.startX * 2
                startY = prev.startY * 2
                stopX = prev.stopX * 2
                stopY = prev.stopY * 2
            self.level_infos.append(self.level_info(startX=startX, startY=startY, stopX=stopX, stopY=stopY,
                                                    matrix_width=matrix_width, matrix_height=matrix_height,
                                                    zoom_level=level,
                                                    pixel_x_size=self.mercator.Resolution(level),
                                                    pixel_y_size=self.mercator.Resolution(level),
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

    def getTileStartStopLL(self, min_x, min_y, max_x, max_y, level):
        if level not in self.levels:
            return (0, 0), (0, 0)
        startX, startY = self.deg2num(min_x, min_y, level)
        stopX, stopY = self.deg2num(max_x, max_y, level)
        return startX, startY, stopX+1, stopY+1

    def getTileStartStopTMS(self, level):
        if level not in self.levels:
            return (0, 0), (0, 0)
        tileStartTMS = self.mercator.MetersToTile(self.min_x, self.min_y, level)
        tileStopTMS = self.mercator.MetersToTile(self.max_x, self.max_y, level)
        startX = tileStartTMS[0]
        startY = tileStartTMS[1]
        stopX = tileStopTMS[0]
        stopY = tileStopTMS[1]
        return startX, startY, stopX+1, stopY+1

    def getTileStartStop(self, level):
        if level not in self.levels:
            return (0, 0), (0, 0)
        tileStartTMS = self.mercator.MetersToTile(self.min_x, self.min_y, level)
        tileStart = self.mercator.GoogleTile(tileStartTMS[0], tileStartTMS[1], level)
        tileStopTMS = self.mercator.MetersToTile(self.max_x, self.max_y, level)
        tileStop = self.mercator.GoogleTile(tileStopTMS[0], tileStopTMS[1], level)
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
        if self.verbose:
            self.checkTiles()
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
        result = self.connection.execute("""SELECT * FROM gpkg_spatial_ref_sys WHERE srs_name=?;""",
                                         (srs_name,)).fetchone()
        if result is None:
            result = self.connection.execute(
                """SELECT MAX(srs_id) AS max_id FROM gpkg_spatial_ref_sys;""").fetchone()
            if result is None:
                return -1
            srs_id = result['max_id']
            srs_id += 1
            self.connection.execute(
                """
                INSERT INTO gpkg_spatial_ref_sys(srs_name, srs_id, organization, organization_coordsys_id, definition)
                            VALUES(?, ?, 'NONE', 0, ?)
                """, (srs_name, self.cache.srs_id, self.cache.wkt))
            self.connection.commit()
            return srs_id
        else:
            return result['srs_id']

    def add_cache(self, path):
        if not self.cache.open(path):
            return False
        identifier = os.path.basename(path)
        table_name = re.sub('[.~,;-]', '', identifier + TILES_TABLE_SUFFIX)
        if not table_name[0].isalpha():
            table_name = TILES_TABLE_PREFIX + table_name
        table_name = table_name.lower()
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
                (table_name, self.cache.srs_id, self.cache.matrix_min_x, self.cache.matrix_min_y, self.cache.matrix_max_x, self.cache.matrix_max_y)
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
                                           (tile_row - level.startY) + level.offset_y,
                                           (tile_column - level.startX) + level.offset_x):
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
