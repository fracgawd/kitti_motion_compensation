# largely inspired by https://docs.docker.com/get-started/

FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND noninteractive
WORKDIR /work_zone

COPY deployment/install_dependencies.sh /work_zone/
RUN /work_zone/install_dependencies.sh

FROM base AS build-library

COPY src/ /work_zone/src/
COPY include/ /work_zone/include/
COPY CMakeLists.txt /work_zone/CMakeLists.txt

RUN cd /work_zone/ && rm -rf build/ && mkdir build/ && cd build/ && cmake .. && make

FROM build-library AS build-and-run-tests

COPY --from=build-library /work_zone/build/ /work_zone/build/
COPY include/ /work_zone/include/
COPY test/ /work_zone/test/
COPY testing_assets/ /work_zone/testing_assets/

RUN cd /work_zone/build/ && cmake -DBUILD_TESTING=ON .. && make && ctest

FROM build-library AS build-and-run-examples

COPY --from=build-library /work_zone/build/ /work_zone/build/
COPY include/ /work_zone/include/
COPY examples/ /work_zone/examples/

RUN cd /work_zone/build/ && cmake -DBUILD_EXAMPLES=ON .. && make

ENTRYPOINT [ "/work_zone/build/motion_compensate_runs" ] 


