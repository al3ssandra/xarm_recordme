FROM alpine:latest
RUN apk update && apk --no-cache add \
    linux-headers \
    cmake \
    g++ \
    wget \
    unzip \
    make \
    libjpeg-turbo-dev \
    libpng-dev \
    tiff-dev \
    libwebp-dev \
    libavc1394-dev \
    v4l-utils-dev \
    ffmpeg-dev \
    gstreamer-dev \
    gst-plugins-base-dev \
    gtk+3.0-dev \
    eigen-dev \
    cppunit-dev \
    git

RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
RUN unzip opencv.zip
RUN mkdir -p opencv_build
RUN cd opencv_build && cmake \
    -D BUILD_LIST=core,videoio,highgui,imgproc,dnn \
    -D WITH_GSTREAMER=ON \
    -D VIDEOIO_PLUGIN_LIST=gstreamer \
    -D WITH_FFMPEG=ON \
    ../opencv-4.x

RUN cd opencv_build && make -j && make install
RUN rm opencv.zip

RUN git clone https://github.com/orocos/orocos_kinematics_dynamics.git
RUN cd orocos_kinematics_dynamics/orocos_kdl && mkdir build
RUN cd orocos_kinematics_dynamics/orocos_kdl/build && cmake .. && make -j && make install
RUN ln -s /usr/include/eigen3/Eigen/ /usr/include/Eigen
# RUN rm -rf opencv-4.x
# RUN apk del wget unzip && rm -rf /var/cache/apk/*
ENTRYPOINT ["tail"]
CMD ["-f","/dev/null"]