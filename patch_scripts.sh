#!/usr/bin/env bash

# export sourced variables
set -a 

# read versions from config file
source cpplibs_version.cfg

# Create scripts from templates and set version numbers from loaded variables
envsubst '$cmake' < script_templates/build_cgal_components.sh.template > build_cgal_components.sh
envsubst '$cmake, $cgal, $boost, $gmp, $mpfr, $eigen, $glpk, $ipopt, $hmetis, $criterion, $scip' < script_templates/dl_and_build_cpplibs.sh.template > dl_and_build_cpplibs.sh
envsubst '$cmake, $cgal, $boost, $gmp, $mpfr, $eigen, $glpk, $ipopt, $hmetis, $criterion, $scip' < script_templates/CMakeLists.txt.template > CMakeLists.txt
