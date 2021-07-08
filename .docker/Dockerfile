FROM ros:melodic-perception

# Install dependencies
RUN apt update && apt install -y \
        libtinyxml-dev \
        python3-pip \
        gcc-8 g++-8 \
        clang-9 clang++-9 clang-format-9 clang-tidy-9

RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir cmake

# Use GCC 8 as default GNU toolchain
RUN sudo update-alternatives --install /usr/bin/gcc          gcc          /usr/bin/gcc-8           8 \
                               --slave /usr/bin/g++          g++          /usr/bin/g++-8

# Use Clang 9 as default LLVM toolchain
RUN sudo update-alternatives --install /usr/bin/clang        clang        /usr/bin/clang-9         9 \
                               --slave /usr/bin/clang++      clang++      /usr/bin/clang++-9         \
                               --slave /usr/bin/clang-format clang-format /usr/bin/clang-format-9    \
                               --slave /usr/bin/clang-tidy   clang-tidy   /usr/bin/clang-tidy-9

# Use colored X terminal emulator
ENV TERM=xterm-color

# Add binary directory of packages installed by pip to PATH
ENV PATH="/root/.local/bin:${PATH}"

# Create default workspace
RUN mkdir /root/catkin_ws

# Start in workspace directory
WORKDIR /root/catkin_ws
