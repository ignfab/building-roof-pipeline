FROM debian:bullseye-slim

# Simple dockerfile to test C++ libraries compilation

RUN apt-get update && apt-get install -y gettext-base build-essential wget m4 xz-utils libssl-dev libtbb-dev libreadline-dev pkg-config liblapack-dev libgsl-dev gfortran libopenblas-dev libgsl-dev libcliquer-dev libopenmpi-dev

WORKDIR /pipeline

COPY script_templates/ script_templates/
COPY cgal_components/ cgal_components/
COPY cpplibs_version.cfg cpplibs_version.cfg 
COPY patch_scripts.sh patch_scripts.sh
RUN bash patch_scripts.sh 
RUN bash dl_and_build_cpplibs.sh -dc
RUN bash build_cgal_components.sh
RUN ls -alh cgal_components
