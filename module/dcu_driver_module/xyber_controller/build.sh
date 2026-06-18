#!/bin/bash
###
 # @Author: richie.li
 # @Date: 2024-10-12 15:45:17
 # @LastEditors: richie.li
 # @LastEditTime: 2024-10-14 15:39:35
### 


cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=./build/install \
    $@

if [ $? -ne 0 ]; then
    echo "cmake failed"
    exit 1
fi

# make
cd build
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "make failed"
    exit 1
fi

# install
if [ -d install ]; then
    rm -rf install
fi

make install

# make package