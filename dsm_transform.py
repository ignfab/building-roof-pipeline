#!/usr/bin/env python3
# coding: utf-8

import sys
from time import time
import numpy as np
from numpy import asarray
from osgeo import gdal, ogr
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import orient
from shapely.wkt import dumps, loads
import os
import shutil
import math

# TODO : Support a debug flag for more log outputs
debug_output = 0

# Test true/false or 0/1 variables
def bool_string(s):
    return s.lower() == "true" or s == "1"

#  return a list of segments from a alinestring or a linearring
def segments(curve):
    return list(map(LineString, zip(curve.coords[:-1], curve.coords[1:])))

# Calculate normal to a segment 
def segment_normal(segment):
    a = Point(segment.coords[0])
    b = Point(segment.coords[1])
    dx = b.x - a.x
    dy = b.y - a.y
    norm =math.sqrt(dx ** 2 + dy ** 2 )
    normal = Point(dy/norm,-dx/norm,0)
    return normal

# Convert X,Y,Z,nx,ny,nz coords into X,Z,Y,nx,nz,ny coords
# Equivalent to a rotation matrix
def xyz_to_xzy(array, compute_normals):
    array[:,[2,1]] = array[:,[1,2]] # gives x y z
    array[:,2] *= -1 # donne x z -y
    if bool_string(compute_normals):
        array[:,[5,4]] = array[:,[4,5]] # gives nx nz ny
        array[:,5] *= -1 # gives nx nz -ny
    return array

# Main method
def run(results_directory: str = "results",
        input_dsm_file: str ="input.tiff",
        input_footprint_file: str ="cutline.geojson",
        output_file: str ="output.ply",
        recenter_model: str ="false",
        crop_feature: str ="false",
        compute_normals: str ="true", 
        include_faces: str ="true",
        transform_walls_floor: str ="true",
        swap_axis: str ="false"):
    
    # Remove results dir if exists
    if not os.path.exists(results_directory):
        os.mkdir(results_directory)

    # Remove result file if it already exists
    output_file_path = os.path.join(results_directory, output_file)
    if not os.path.exists(results_directory):
        os.remove(output_file_path)

    # start timestamp
    start = time()

    # Read DSM
    dsm_ds = gdal.Open(input_dsm_file)
    # Read scaled footprint
    scaled_footprint_ds = ogr.Open(input_footprint_file)
    open_ = time()
    # Set variables
    transform = dsm_ds.GetGeoTransform()
    width = dsm_ds.RasterXSize
    height = dsm_ds.RasterYSize
    dsm_xmin = transform[0]
    dsm_ymax = transform[3]
    dsm_xmax = dsm_xmin + transform[1] * width
    dsm_ymin = dsm_ymax + transform[5] * height
    no_data = dsm_ds.GetRasterBand(1).GetNoDataValue()
    print(f"DSM input size is {width}x{height} pixels")
    # Get elevation values from DSM as numpy array
    dsm_values = dsm_ds.ReadAsArray()
    # vertext indices initialization with -1 value
    vertex_indices = np.full((height, width), -1)
    init = time()
    faces = None

    ############################
    # Step 0 : Vertex indexing #
    ############################

    begin_vertex = time()
    # In case no_data valued vertices must be ignored
    if bool_string(crop_feature):
        # Select vertices indices where value is different from no data value
        with_data_indices = np.nonzero(dsm_values != no_data)
        # Get elevation values based on this selection
        altitude_values = dsm_values[with_data_indices]
        # Create an index list for these values 
        with_data_index_list = np.arange(with_data_indices[0].shape[0])
        # Insert into vertex_indices the 
        vertex_indices[with_data_indices] = with_data_index_list
    else:
        # Do the same without ignoring no_data values
        with_data_indices = (np.repeat(np.arange(height), width),
                             np.tile(np.arange(width), height))
        altitude_values = dsm_values[with_data_indices]
        with_data_index_list = np.arange(with_data_indices[0].shape[0])
        vertex_indices[with_data_indices] = with_data_index_list

    # Create vertices array
    zeros = np.zeros_like(altitude_values)
    ones = np.ones_like(altitude_values)

    # If recenter_model get coordinates into a local systems
    if bool_string(recenter_model):
        vertex = np.vstack(( with_data_indices[1] * transform[1], 
                             with_data_indices[0] * transform[5], 
                             altitude_values,
                             zeros,
                             zeros,
                             ones ))
    # TODO : Support another option to set model in ellispoidal coordinates 
    else:
        vertex = np.vstack((transform[0] + with_data_indices[1] * transform[1], 
                           transform[3] + with_data_indices[0] * transform[5], 
                           altitude_values, 
                           zeros, 
                           zeros, 
                           ones))

    vertex = np.transpose(vertex)

    end_vertex = time()

    if(bool_string(swap_axis)):
        vertex = xyz_to_xzy(vertex, compute_normals)

    # Remove default normals if no normal is needed
    if not bool_string(compute_normals):
         vertex = vertex[:,:3]
        
    if bool_string(include_faces):

        ####################################
        # Step 1 : Roof faces calcultation #           
        ####################################

        begin_faces = time()
        # Using numpy broadcasting we compute indices blocks for a sliding window
        # of size 2x2
        raw_sliding_windows_indices = np.array(
            [0, 1, width, width+1]) + np.arange(width*(height-1))[:, None]
        #print("raw_sliding_windows_indices : " + str(raw_sliding_windows_indices.shape))

        # Delete sliding windows that are out of bounds
        sliding_windows_indices = np.delete(raw_sliding_windows_indices, np.arange(
            width-1, raw_sliding_windows_indices.shape[0], width), axis=0)
        #print("sliding_windows_indices : " + str(sliding_windows_indices.shape))

        # Get vertex indices for each sliding window block
        vertex_indices_flat = vertex_indices.flatten()
        raw_windowed_vertex_indices = vertex_indices_flat[sliding_windows_indices]
        #print("raw_windowed_vertex_indices : " +str(raw_windowed_vertex_indices.shape))

        # We differentiate between two cases:
        # * Indices blocks where all vertex indices are positive
        # * Indices blocks where only one vertex index is negative
        filter = np.count_nonzero(raw_windowed_vertex_indices == -1, axis=1)
        windowed_vertex_indices_all_positive = raw_windowed_vertex_indices[filter < 1]
        windowed_vertex_indices_only_one_negative = raw_windowed_vertex_indices[filter == 1]
        #print("windowed_vertex_indices_all_positive : " +str(windowed_vertex_indices_all_positive.shape))
        #print("windowed_vertex_indices_only_one_negative : " +str(windowed_vertex_indices_only_one_negative.shape))

        # First case: Indices blocks where all vertex indices are positive
        # Get vertices x,y,z coordinates
        svp = vertex[windowed_vertex_indices_all_positive][:, :, :3]
        #print("svp : " + str(svp.shape))
        # Calculate distance between vertices
        dst = np.square(np.linalg.norm(svp[:, 0, :]-svp[:, 2, :], axis=1))-np.square(
            np.linalg.norm(svp[:, 1, :]-svp[:, 3, :], axis=1))
        #print("dst : " + str(dst.shape))
        # Create faces
        case_1 = windowed_vertex_indices_all_positive[np.where(dst <= 0)]
        #print("case_1 : " + str(case_1.shape))
        face_1 = np.vstack(
            (case_1[:, 0], case_1[:, 2], case_1[:, 3], case_1[:, 0], case_1[:, 3], case_1[:, 1]))
        face_1 = np.transpose(face_1).reshape(-1, 3)
        #print("face_1 : " + str(face_1.shape))
        case_2 = windowed_vertex_indices_all_positive[np.where(dst > 0)]
        #print("case_2 : " + str(case_2.shape))
        face_2 = np.vstack(
            (case_2[:, 0], case_2[:, 2], case_2[:, 1], case_2[:, 1], case_2[:, 2], case_2[:, 3]))
        face_2 = np.transpose(face_2).reshape(-1, 3)
        #print("face_2 : " + str(face_2.shape))
    

        # Second case: Indices blocks where only one vertex index is negative
        # Get vertices x,y,z coordinates
        m, n = windowed_vertex_indices_only_one_negative.shape
        # Reorder indices to always be in clockwise order configuration
        no_to_reorder = windowed_vertex_indices_only_one_negative[
            windowed_vertex_indices_only_one_negative[:, 0] == -1, :]
        to_reorder = windowed_vertex_indices_only_one_negative[
            windowed_vertex_indices_only_one_negative[:, 0] != -1, :]
        order = [0, 2, 3, 1]
        reordered = to_reorder[:, order]
        # Create faces
        face_3_1 = no_to_reorder[no_to_reorder != -1].reshape(-1, n-1)
        face_3_2 = reordered[reordered != -1].reshape(-1, n-1)

        # Concatenate all faces
        faces = np.concatenate((face_1, face_2, face_3_1, face_3_2))

        end_faces = time()

    ###############################################
    # Step 2 : Roof vertices normal calculation #
    ###############################################

    if bool_string(compute_normals):

        begin_normals = time()

        # Add a border of -2 indices around the original array
        padded_vertex_indices = np.pad(
            vertex_indices, ((1, 1), (1, 1)), constant_values=((-2, -2),))
        # print("padded_vertex_indices : " + str(padded_vertex_indices.shape))

        # We use here a diamond shaped sliding window
        h, w = padded_vertex_indices.shape
        raw_padded_window_indices = np.array(
            [w, 0, w-1, 2*w, w+1]) + np.arange(w*(h-2))[:, None]
        # print("raw_padded_window_indices : " + str(raw_padded_window_indices.shape))

        # Delete first and last colum of each row
        deletion_index = np.array([0, w-1]) + np.arange(0, w*(h-2), w)[:, None]
        deletion_index = deletion_index.flatten()
        padded_window_indices = np.delete(
            raw_padded_window_indices, deletion_index, axis=0)
        # print("padded_window_indices : " + str(padded_window_indices.shape))

        # Extract vertices index list for each sliding window
        windowed_padded_vertex_indices = padded_vertex_indices.flatten()[
            padded_window_indices]
        # print("windowed_padded_vertex_indices : " + str(windowed_padded_vertex_indices.shape))
        # print(windowed_padded_vertex_indices[0])


        # At this point we now have everything to calculate the normal for each point
        # If we encounter -2 or -1 indices we know we can ignore them
        # Lets perform a dot product to calcule normals

        # Resulting normal array
        normal = np.zeros(3, dtype=float)
        zero_normals = np.tile(normal, (vertex.shape[0], 1))

        # We only consider valid "center" point for normal calculation (different from -2 or -1)
        wpv_has_center = windowed_padded_vertex_indices[windowed_padded_vertex_indices[:, 0] >= 0, :]
        #print("wpv_has_center : " + str(wpv_has_center.shape))

        # We then gather valid surrounding points
        wpv_north_west = wpv_has_center[(wpv_has_center[:, 1] >= 0) & (
            wpv_has_center[:, 2] >= 0), :]
        wpv_west_south = wpv_has_center[(wpv_has_center[:, 2] >= 0) & (
            wpv_has_center[:, 3] >= 0), :]
        wpv_south_east = wpv_has_center[(wpv_has_center[:, 3] >= 0) & (
            wpv_has_center[:, 4] >= 0), :]
        wpv_east_north = wpv_has_center[(wpv_has_center[:, 1] >= 0) & (
            wpv_has_center[:, 4] >= 0), :]

        # We use numpy to perform dot product
        cross_north_west = np.cross(
            vertex[wpv_north_west[:, 1]][:, :3], vertex[wpv_north_west[:, 2]][:, :3])
        cross_west_south = np.cross(
            vertex[wpv_west_south[:, 2]][:, :3], vertex[wpv_west_south[:, 3]][:, :3])
        cross_south_east = np.cross(
            vertex[wpv_south_east[:, 3]][:, :3], vertex[wpv_south_east[:, 4]][:, :3])
        cross_east_north = np.cross(
            vertex[wpv_east_north[:, 4]][:, :3], vertex[wpv_east_north[:, 1]][:, :3])

        # Add to the result normals array
        zero_normals[wpv_north_west[:, 0]] += cross_north_west
        zero_normals[wpv_west_south[:, 0]] += cross_west_south
        zero_normals[wpv_south_east[:, 0]] += cross_south_east
        zero_normals[wpv_east_north[:, 0]] += cross_east_north
        norms = np.linalg.norm(zero_normals, axis=1)
        norms[norms == 0] = 1
        normals = zero_normals / norms[:, None]

        vertex[:, 3:] = normals

        end_normals = time()

    ##################################
    # Step 3 : Create roof and walls #
    ##################################

    if bool_string(transform_walls_floor):

        # Use gdal to general vertices from footprint shape
        footprint_lyr = scaled_footprint_ds.GetLayer()
        # TODO: Handle error in case of a multipolygon
        footprint = footprint_lyr.GetNextFeature()
        # Get roof altitude from IGN BDTOPO
        alti_min_sol = footprint.GetFieldAsString("altitude_minimale_sol")
        try:
            alti_min_sol=float(alti_min_sol)
        except ValueError:
            print("alti_min_sol unavailable. Process will terminate")
            sys.exit(2)

        ##################
        # floor creation #
        ##################
        
        begin_floor = time()

        x_min, x_max, y_min, y_max = footprint_lyr.GetExtent()

        # Raster memory creation
        # The DSM resolution is used for the new raster
        pixel_size = abs(transform[5])
        x_res = int((x_max - x_min) / pixel_size)
        y_res = int((y_max - y_min) / pixel_size)
        raster_mem_ds = gdal.GetDriverByName('MEM').Create('', x_res, y_res, gdal.GDT_Byte)
        raster_mem_ds.SetGeoTransform((x_min, pixel_size, 0, y_max, 0, -pixel_size))
        geotransform = raster_mem_ds.GetGeoTransform()
        band = raster_mem_ds.GetRasterBand(1)
        band.SetNoDataValue(255)

        # Polygon rasterization
        gdal.RasterizeLayer(raster_mem_ds, [1], footprint_lyr, burn_values=[1])

        # Read raster as numpy array
        array = band.ReadAsArray()

        # Get indices with value
        floor_data_indices = np.nonzero(array == 1)

        # Vertices array creation
        altis_min_sol = np.ones_like(floor_data_indices[0]) * float(alti_min_sol)
        floor_points = np.vstack(( floor_data_indices[1] * geotransform[1], 
                                   floor_data_indices[0] * geotransform[5], 
                                   altis_min_sol ))
        floor_points = np.transpose(floor_points)

        # Add normals if necessary
        if bool_string(compute_normals):
            vertical_normal = np.array([0,0,-1])
            floor_normals = np.tile(vertical_normal, (len(floor_points), 1))
            floor_vertices = np.concatenate((floor_points,floor_normals),axis=1)
        else:
            floor_vertices = floor_points

        if(bool_string(swap_axis)):
            floor_vertices = xyz_to_xzy(floor_vertices, compute_normals)

        end_floor = time()

        ##################
        # walls creation #
        ##################

        begin_walls = time()

        multipolygons = loads(footprint.GetGeometryRef().ExportToWkt())
        # TODO: Handle error in case of a multipolygons
        raw_polygon = multipolygons.geoms[0]
        polygon = orient(raw_polygon)
        linearring = polygon.exterior
        ring_segments = segments(linearring)
        xmin = linearring.bounds[0]
        ymin = linearring.bounds[1]
        xmax = linearring.bounds[2]
        ymax = linearring.bounds[3]

        add_distance = pixel_size
        distance = 0

        # Using numpy to be faster than with loops
        wall_top_pts = []
        wall_lines = []

        while distance < linearring.length:
            seed = linearring.interpolate(distance)
            segment = 0
            # TODO : Handle borderline error cases
            for s in ring_segments:
                if s.distance(seed) < 1e-8:
                    segment = s
                    break
            normal = segment_normal(segment)
            wall_top_pts.append(np.array([seed.x,seed.y,seed.z,normal.x,normal.y,normal.z]))
            distance += add_distance

        for p in wall_top_pts:
            height = p[2] - float(alti_min_sol)
            repeats = int(height / add_distance)
            decrements = np.arange(repeats+1)*add_distance*-1
            wall_line = np.tile(p,(repeats+1,1))
            wall_line[:,2] = wall_line[:,2]+decrements
            wall_lines.append(wall_line)
        
        np_wall_points = np.vstack(wall_lines)

        # Shift to local coordinates
        origin = np.array([xmin,ymax,0,0,0,0])
        np_wall_points = np_wall_points - origin

        # Compute normals
        if bool_string(compute_normals):
            wall_vertices = np_wall_points
        else:
            wall_vertices = np_wall_points[:,:3]
            
        if(bool_string(swap_axis)):
            wall_vertices = xyz_to_xzy(wall_vertices, compute_normals)

        # Merge floor and walls vertices
        building_vertices = np.concatenate((floor_vertices,wall_vertices),axis=0)
        end_walls = time()

        # Merge floor and walls vertices with building vertices
        vertex = np.concatenate((vertex,building_vertices))

    if bool_string(recenter_model):
        vertex[:,1] -= vertex[:,1].min()

    if bool_string(compute_normals):
        template = """ply
format ascii 1.0
element vertex {nvertices:n}
property float x
property float y
property float z
property float nx
property float ny
property float nz
element face {nfaces:n}
property list int int vertex_index
end_header
"""
    else:
        template = """ply
format ascii 1.0
element vertex {nvertices:n}
property float x
property float y
property float z
element face {nfaces:n}
property list int int vertex_index
end_header
"""

    context = {
        "nvertices": len(vertex),
        "nfaces": len(faces) if faces is not None else 0
    }

    begin_write = time()
    # Pretty slow but hard to something faster
    with open(output_file_path, "wb") as outfile:
        tpl = template.format(**context)
        outfile.write(tpl.encode())
        np.savetxt(outfile, vertex, fmt="%.6f")
        if bool_string(include_faces):
            np.savetxt(outfile, faces, fmt="3 %i %i %i")
    end_write = time()

    print("Time elapsed in each task :")
    print(f"    Open: {open_ - start}")
    print(f"    Init: {init - open_}")
    print(f"    Vertex: {end_vertex - begin_vertex}")
    if bool_string(include_faces):
        print(f"    Faces: {end_faces - begin_faces}")
    if bool_string(compute_normals):
        print(f"    Normals: {end_normals - begin_normals}")
    if bool_string(transform_walls_floor):
        print(f"    Floor: {end_floor - begin_floor}")
        print(f"    Walls: {end_walls - begin_walls}")
    print(f"    Write: {end_write - begin_write}")
    print(f"    Total: {end_write - start}")
    print(f"Successfully transformed \"{input_dsm_file}\" into \"{output_file_path}\"!\n")
