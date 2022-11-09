#!/usr/bin/env python3
# coding: utf-8

from osgeo import ogr, gdal
from shapely import affinity
from shapely.wkt import loads
import shutil
import os
import time

# Get building footprint from IGNF WFS for a given IGNF building identifier
def get_footprint_from_wfs(ignf_building_id,
                            output_filepath,
                            retry_count,
                            retry_interval, 
                            timeout):
    wfs_building_fid_url = f"https://wxs.ign.fr/essentiels/geoportail/wfs?request=GetFeature" \
                            f"&typenames=BDTOPO_V3:batiment&outputFormat=json&srsName=EPSG:2154" \
                            f"&featureID={ignf_building_id}"
    # gdal/ogr http query timeout
    gdal.SetConfigOption('GDAL_HTTP_TIMEOUT', str(timeout))
    # get from url
    for i in range(0,retry_count):
        try:
            ogr_ds = ogr.Open(wfs_building_fid_url)
            break
        except Exception as e:
            print("WFS request failed. Will retry")
            print(e)
            time.sleep(retry_interval)
            continue
    if not ogr_ds:
        raise Exception("WFS request failed")
    # building layer
    building_lyr = ogr_ds.GetLayer()
    # get ogr driver   
    geojson_drv = ogr.GetDriverByName("GeoJSON")
    output_ds = geojson_drv.CreateDataSource(output_filepath)
    output_ds.CopyLayer(building_lyr, "footprint")
    del ogr_ds
    del output_ds
    return 1

# Get DSM extract from IGNF WMS
def get_dsm_from_wms(output_filename, 
                        gdal_driver, 
                        layer_name, 
                        output_format, 
                        upper_left_x, 
                        upper_left_y, 
                        lower_right_x, 
                        lower_right_y, 
                        width, 
                        height, 
                        retry_count, 
                        retry_interval, 
                        timeout):

    # WMS XML the gdal way
    wms = f"""<GDAL_WMS>
    <Service name="WMS">
        <Version>1.3.0</Version>
        <ServerUrl>https://wxs.ign.fr/altimetrie/geoportail/r/wms?</ServerUrl>
        <Layers>{layer_name}</Layers>
        <ImageFormat>{output_format}</ImageFormat>
        <CRS>EPSG:2154</CRS>
        <Styles></Styles>
    </Service>
    <DataWindow>
        <UpperLeftX>{upper_left_x}</UpperLeftX>
        <UpperLeftY>{upper_left_y}</UpperLeftY>
        <LowerRightX>{lower_right_x}</LowerRightX>
        <LowerRightY>{lower_right_y}</LowerRightY>
        <SizeX>{width}</SizeX>
        <SizeY>{height}</SizeY>
    </DataWindow>
    <Projection>EPSG:2154</Projection>
    <BandsCount>1</BandsCount>
    <DataType>Float32</DataType>
    <UnsafeSSL>true</UnsafeSSL>
    <Timeout>{timeout}</Timeout>
    </GDAL_WMS>
    """

    # get DSM from WMS
    for i in range(0,retry_count):
        try:
            gdal_ds = gdal.Open(wms)
        except:
            print("WMS request failed. Will retry")
            time.sleep(retry_interval)
            continue
    if not gdal_ds:
        raise Exception("WMS request failed")

    # get gdal driver   
    gdal_drv = gdal.GetDriverByName(gdal_driver)
    gdal_drv.CreateCopy(output_filename, gdal_ds, 0)
    del gdal_ds
    return 1

# Import DSM and crop it using building footprint
def run(ign_building_id: str = "BATIMENT0000000356564051", 
        working_directory: str = "dsm_import",   
        footprint_filename: str = "footprint.geojson", 
        scaled_footprint_filename: str = "scaled_footprint.geojson",
        dsm_filename: str = "downloaded_dsm.tiff",
        result_filename: str = "cropped_dsm.tiff",
        srs: str = "2154",
        footprint_scaling_factor: str = "1.0",
        dsm_pixel_resolution: str = "0.25",
        nodata_value: str = None,
        http_timeout: str = "5",
        http_retry_count: str = "5",
        http_retry_interval: str = "2"):

    ##################
    # init variables #
    ##################

    # srs definition for french legal reference system
    lamb93_srs = ogr.osr.SpatialReference()
    lamb93_srs.ImportFromEPSG(int(srs))

    # filepaths
    footprint_filepath = os.path.join(working_directory, footprint_filename)
    scaled_footprint_filepath = os.path.join(working_directory, scaled_footprint_filename)
    dsm_filepath = os.path.join(working_directory, dsm_filename)
    result_filepath = os.path.join(working_directory, result_filename)

    ##########################
    # gdal/org configuration #
    ##########################

    gdal.UseExceptions()
    ogr.UseExceptions()
 
    #########################################
    # delete and recreate working directory #
    #########################################

    if os.path.exists(working_directory) and os.path.isdir(working_directory):
        shutil.rmtree(working_directory)
    os.mkdir(working_directory)

    ##########################
    # Get building footprint #
    ##########################

    try: 
        get_footprint_from_wfs(ign_building_id, footprint_filepath, int(http_retry_count), float(http_retry_interval), float(http_timeout))
    except: 
        raise Exception ("Could not get building footprint from WFS")

    ##############################
    # Rescale building footprint #
    ##############################

    # reopen footprint file
    footprint_ds = ogr.Open(footprint_filepath)
    footprint_layer = footprint_ds.GetLayer("footprint")

    # create new file for scaled footprint
    geojson_drv = ogr.GetDriverByName("GeoJSON")
    scaled_footprint_ds = geojson_drv.CreateDataSource(
        scaled_footprint_filepath)
    scaled_layer = scaled_footprint_ds.CreateLayer(
        "scaled_footprint", lamb93_srs, footprint_layer.GetLayerDefn().GetGeomType())
    feature_defn = scaled_layer.GetLayerDefn()

    # iterate over footprint parts
    for feature in footprint_layer:
        # export geometry to shapely
        geom_wkt = feature.geometry().ExportToWkt()
        geom = loads(geom_wkt)
        # scale footprint part using shapely
        scaled_geom = affinity.scale(geom, xfact=float(
            footprint_scaling_factor), yfact=float(footprint_scaling_factor))
        scaled_feature = feature.Clone()
        # export scaled geometry back to OGR
        scaled_geom_ogr = ogr.CreateGeometryFromWkb(scaled_geom.wkb)
        # Add scaled geometry to layer
        feature.SetGeometry(scaled_geom_ogr)
        scaled_layer.CreateFeature(feature)
        feature = None

    # close data sources
    del footprint_ds
    del scaled_footprint_ds

    ################
    # Get DSM data #
    ################

    # Reopen scaled footprint file and use its extent as building extent
    scaled_footprint_ds = ogr.Open(scaled_footprint_filepath)
    scaled_footprint_lyr = scaled_footprint_ds.GetLayer("scaled_footprint")
    building_extent = scaled_footprint_lyr.GetExtent()

    # Calculate image width and height
    width = int((building_extent[1] - building_extent[0]) //
                float(dsm_pixel_resolution))
    height = int((building_extent[3] - building_extent[2]) //
                 float(dsm_pixel_resolution))
    try: 
        get_dsm_from_wms(dsm_filepath, 
                    "GTiff", 
                    "ELEVATION.ELEVATIONGRIDCOVERAGE.HIGHRES.MNS", 
                    "image/geotiff", 
                    building_extent[0], 
                    building_extent[3], 
                    building_extent[1], 
                    building_extent[2], 
                    width, 
                    height, 
                    int(http_retry_count), 
                    float(http_retry_interval), 
                    float(http_timeout))
    except: 
        raise Exception ("Could not get DSM from WMS")

    ###############################
    # Cut DSM with scaled footprint #
    ###############################

    min_dsm_value = gdal.Info(dsm_filepath, format="json", computeMinMax=True)[
        "bands"][0]["computedMin"]

    if not nodata_value:
        nodata_value = min_dsm_value

    gdal.Warp(result_filepath, dsm_filepath, format="GTiff",
              cutlineDSName=scaled_footprint_filepath, cropToCutline=True,
              dstNodata=float(nodata_value), dstSRS="EPSG:2154")
    print(
        f"Successfully imported data from building \"{ign_building_id}\" into \"{result_filepath}\"!\n")
