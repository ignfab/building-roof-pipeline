#!/usr/bin/env python3
# coding: utf-8

import configparser
from functools import total_ordering
import dsm_import
import dsm_transform
import argparse
import subprocess
import shutil
import sys, os

parser = argparse.ArgumentParser(description='Roof reconstruction pipeline')
parser.add_argument("-c", "--clean", action='store_true')
parser.add_argument("-b", "--bid", help='IGNF building identifier')
parser.add_argument("-f", "--file", help='pipeline configuration .ini file')
args = parser.parse_args()

config = configparser.ConfigParser()

if(args.file):
    config.read(args.file)
else:
    try:
        config.read('default.ini')
    except IOError:
        sys.exit("Configuration file not found. Exiting pipeline.") 

if (args.clean):
    if config.has_section('DSM_IMPORT'):
        shutil.rmtree(config['DSM_IMPORT']['working_directory'], ignore_errors=True)
    if config.has_section('DSM_TRANSFORM_POINTS'):
        shutil.rmtree(config['DSM_TRANSFORM_POINTS']['results_directory'], ignore_errors=True)
    if config.has_section('DSM_TRANSFORM_MESH'):
        shutil.rmtree(config['DSM_TRANSFORM_MESH']['results_directory'], ignore_errors=True)
    if config.has_section('DSM_TRANSFORM_POINTS_WALLS'):
        shutil.rmtree(config['DSM_TRANSFORM_POINTS_WALLS']['results_directory'], ignore_errors=True)

if(args.bid):
    building_id = str(args.bid)
else:
    if config.has_section('DSM_IMPORT') and config.has_option('DSM_IMPORT','ign_building_id'):
        building_id = config['DSM_IMPORT']['ign_building_id']
    else:
        sys.exit("No IGNF building ID found. Exiting pipeline.")
    
##############
# Import DSM #
##############

if config.has_section('DSM_IMPORT'):
    p = config['DSM_IMPORT']
    dsm_import.run(
        building_id,
        p['working_directory'],
        p['cutline_filename'],
        p['scaled_cutline_filename'],
        p['dsm_filename'],
        p['result_filename'],
        p['srs'],
        p['footprint_scaling_factor'],
        p['dsm_pixel_resolution'],
        p['nodata_value'],
        p['http_timeout'],
        p['http_retry_count'],
        p['http_retry_interval']
    )

#################
# Transform DSM #
#################

if config.has_section('DSM_TRANSFORM_POINTS'):
    p = config['DSM_TRANSFORM_POINTS']
    dsm_transform.run(
        p['results_directory'],
        p['input_dsm_file'],
        p['input_footprint_file'],
        p['output_file'],
        p['recenter_model'],
        p['crop_feature'],
        p['compute_normals'],
        p['include_faces'],
        p['add_walls_and_floor'],
        p['swap_axis']
    )

if config.has_section('DSM_TRANSFORM_MESH'):
    p = config['DSM_TRANSFORM_MESH']
    dsm_transform.run(
        p['results_directory'],
        p['input_dsm_file'],
        p['input_footprint_file'],
        p['output_file'],
        p['recenter_model'],
        p['crop_feature'],
        p['compute_normals'],
        p['include_faces'],
        p['add_walls_and_floor'],
        p['swap_axis']
    )

if config.has_section('DSM_TRANSFORM_POINTS_WALLS'):
    p = config['DSM_TRANSFORM_POINTS_WALLS']
    dsm_transform.run(
        p['results_directory'],
        p['input_dsm_file'],
        p['input_footprint_file'],
        p['output_file'],
        p['recenter_model'],
        p['crop_feature'],
        p['compute_normals'],
        p['include_faces'],
        p['add_walls_and_floor'],
        p['swap_axis']
    )

#######################
# Run CGAL components #
#######################

if config.has_section('EDGE_COLLAPSE'):
    p = config['EDGE_COLLAPSE']
    subprocess.run(["cmake-build/bin/edge-collapse",
                    p['edge_collapse_input_file'],
                    p['edge_collapse_output_file'],
                    p['edge_collapse_ratio']
                    ])

if config.has_section('STRUCTURING'):
    p = config['STRUCTURING']
    subprocess.run(["cmake-build/bin/structuring",
                    p['structuring_input_file'],
                    p['structuring_output_file'],
                    p['structuring_epsilon']
                    ])

if config.has_section('RANSAC'):
    p = config['RANSAC']
    subprocess.run(["cmake-build/bin/ransac",
                    p['ransac_input_file'],
                    p['ransac_output_file'],
                    p['ransac_probability'],
                    p['ransac_min_points_percentage'],
                    p['ransac_epsilon'],
                    p['ransac_cluster_epsilon'],
                    p['ransac_normal_threshold'],
                    p['ransac_iteration_count']
                    ])

if config.has_section('REGION-GROWING-MESH'):
    p = config['REGION-GROWING-MESH']
    subprocess.run(["cmake-build/bin/region-growing-mesh",
                    p['region_growing_mesh_input_file'],
                    p['region_growing_mesh_output_file'],
                    p['region_growing_mesh_max_distance_to_plane'],
                    p['region_growing_mesh_max_accepted_angle'],
                    p['region_growing_mesh_min_region_size']
                    ])

if config.has_section('REGION-GROWING-POINTS'):
    p = config['REGION-GROWING-POINTS']
    subprocess.run(["cmake-build/bin/region-growing-points",
                    p['region_growing_points_input_file'],
                    p['region_growing_points_output_file'],
                    p['region_growing_points_max_distance_to_plane'],
                    p['region_growing_points_max_accepted_angle'],
                    p['region_growing_points_min_region_size'],
                    p['region_growing_points_k']
                    ])


if config.has_section('PSR-RANSAC'):
    p = config['PSR-RANSAC']
    subprocess.run(["cmake-build/bin/psr-ransac",
                    p['psr_ransac_input_file'],
                    p['psr_ransac_output_file'],
                    p['psr_ransac_output_candidate_faces_file'],
                    p['psr_ransac_wt_fitting'],
                    p['psr_ransac_wt_coverage'],
                    p['psr_ransac_wt_complexity'],
                    p['psr_ransac_probability'],
                    p['psr_ransac_min_points_percentage'],
                    p['prs_ransac_epsilon'],
                    p['psr_ransac_cluster_epsilon'],
                    p['psr_ransac_normal_threshold'],
                    p['psr_ransac_iteration_count']
                    ])

if config.has_section('PSR-REGION-GROWING'):
    p = config['PSR-REGION-GROWING']
    subprocess.run(["cmake-build/bin/psr-region-growing",
                    p['psr_region_growing_input_file'],
                    p['psr_region_growing_output_file'],
                    p['psr_region_growing_output_candidate_faces_file'],
                    p['psr_region_growing_wt_fitting'],
                    p['psr_region_growing_wt_coverage'],
                    p['psr_region_growing_wt_complexity'],
                    p['psr_region_growing_max_distance_to_plane'],
                    p['psr_region_growing_max_accepted_angle'],
                    p['psr_region_growing_min_region_size_percentage'],
                    p['psr_region_growing_k']
                    ])
