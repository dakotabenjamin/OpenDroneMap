FROM ubuntu:16.04
MAINTAINER Alex Hagiopol <alex.hagiopol@icloud.com>

# Env variables
ENV DEBIAN_FRONTEND noninteractive

#Install dependencies
#Required Requisites
RUN apt-get update
RUN apt-get update \
    && apt-get install -y -qq --no-install-recommends \
       build-essential \
       cmake \
       git \
       python-pip \
       libgdal-dev \
       gdal-bin \
       libgeotiff-dev \
       pkg-config \
       software-properties-common \
       python-software-properties \
       libgtk2.0-dev \
       libavcodec-dev \
       libavformat-dev \
       libswscale-dev \
       python-dev \
       python-numpy \
       libtbb2 \
       libtbb-dev \
       libjpeg-dev \
       libpng-dev \
       libtiff-dev \
       libjasper-dev \
       libflann-dev \
       libproj-dev \
       libxext-dev \
       liblapack-dev \
       libeigen3-dev \
       libvtk5-dev \
       python-networkx \
       libgoogle-glog-dev \
       libsuitesparse-dev \
       libboost-filesystem-dev \
       libboost-iostreams-dev \
       libboost-regex-dev \
       libboost-python-dev \
       libboost-date-time-dev \
       libboost-thread-dev \
       python-pyproj \
       python-empy \
       python-nose \
       python-pyside \
       python-pyexiv2 \
       python-scipy \
       jhead \
       liblas-bin \
       python-matplotlib \
       libatlas-base-dev \
       libatlas3-base \
    && apt-get remove libdc1394-22-dev \
    && apt-get clean \
RUN pip install -U PyYAML \
                    exifread \
                    gpxpy \
                    xmltodict \
                    catkin-pkg

ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python2.7/dist-packages"
ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/src/opensfm"
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"

# Prepare directories
RUN mkdir /code
WORKDIR /code

# Copy repository files
COPY ccd_defs_check.py /code/ccd_defs_check.py
COPY CMakeLists.txt /code/CMakeLists.txt
COPY configure.sh /code/configure.sh
COPY .gitignore /code/.gitignore
COPY .gitmodules /code/.gitmodules
COPY /modules/ /code/modules/
COPY /opendm/ /code/opendm/
COPY /patched_files/ /code/patched_files/
COPY run.py /code/run.py
COPY /scripts/ /code/scripts/
COPY /SuperBuild/cmake/ /code/SuperBuild/cmake/
COPY /SuperBuild/CMakeLists.txt /code/SuperBuild/CMakeLists.txt
COPY /tests/ /code/tests/

#Compile code in SuperBuild and root directories
RUN cd SuperBuild && mkdir build && cd build && cmake .. && make -j$(nproc) \
    && cd ../.. && mkdir build && cd build && cmake .. && make -j$(nproc) \
    && cd ../SuperBuild/src \
    && ls | grep -v opensfm | parallel rm -rf \
    && cd /code/SuperBuild/build \
    && ls -d */ | grep -v CMake | grep -v pdal | parallel rm -rf && apt-get remove parallel

# Entry point
ENTRYPOINT ["python", "/code/run.py", "--project-path", "/code/"]
