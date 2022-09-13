#!/bin/bash

# Python dependencies
pip install -r requirements.txt

# C++ code
mkdir -p release
cmake -Brelease -Shgs_vrptw -DCMAKE_BUILD_TYPE=Release
cd release && make && cd ..
