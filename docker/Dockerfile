FROM debian:12.1-slim 

# install pcl and compilation tools
RUN apt update && apt install -y git cmake build-essential libpcl-dev

# Install catch2
RUN git clone https://github.com/catchorg/Catch2.git &&\
    cd Catch2 &&\
    git checkout v3.1.1 &&\
    mkdir build &&\
    cd build &&\
    cmake .. &&\
    make &&\
    make install 

# Install nanoflann 
ARG NANOFLANN_VERSION
RUN git clone https://github.com/jlblancoc/nanoflann.git &&\
    cd nanoflann &&\
    git checkout "v${NANOFLANN_VERSION}" &&\
    mkdir build &&\
    cd build &&\
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="/usr/local/nanoflann_v${NANOFLANN_VERSION}" .. &&\
    make &&\
    make install

# compile
COPY . . 
RUN mkdir build && cd build &&\
    cmake -DBUILD_TESTS=ON -DNANOFLANN_VERSION="${NANOFLANN_VERSION}" -DNANOFLANN_INSTALL_PREFIX="/usr/local/nanoflann_v${NANOFLANN_VERSION}" .. &&\
    cmake --build . &&\
    ctest -V -R test --output-on-failure &&\
    cmake --build . --config Release --target install -- -j $(nproc)

# compile example and test install target while doing that
WORKDIR example
RUN mkdir build && cd build &&\
    cmake -DNANOFLANN_VERSION="${NANOFLANN_VERSION}" -DNANOFLANN_INSTALL_PREFIX="/usr/local/nanoflann_v${NANOFLANN_VERSION}" .. &&\
    cmake --build . &&\
    ./nanoflann_pcl_example

# cleanup
RUN rm -rf Catch2 nanoflann nvim-linux64.tar.gz
