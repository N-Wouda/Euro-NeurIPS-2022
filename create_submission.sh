#!/bin/bash

DATE=$(date "+%Y-%m-%d-%H-%M-%S")
mkdir -p tmp/submissions

# C++ source and header files (incl. pybind11)
find hgs_vrptw \
  -wholename "*/pybind11/tests/*" -prune -o -wholename "*/pybind11/build/*" -prune \
  -o \( -name "*.cpp" -o -name "*.h" -o -name "CMakeLists.txt" -o -name "*.cmake" \) \
  -exec zip -o -r "tmp/submissions/$DATE.zip" {} +

# Python code (utilities and various solver strategies + configs)
zip -o -r "tmp/submissions/$DATE.zip" strategies/*
zip -o -r "tmp/submissions/$DATE.zip" configs/solver.toml
zip -o -r "tmp/submissions/$DATE.zip" solver.py tools.py environment.py hgspy.py

# Required CodaLab file
cat > metadata <<- META
description: This file is required to indicate a code submission on the CodaLab platform. There is no need to change this file.
META

zip -o -r "tmp/submissions/$DATE.zip" metadata
rm metadata

# Python dependencies need to be provided as a requirements.txt, since we can
# only use pip in the competition system.
cat > requirements.txt <<- REQ
numpy == 1.22.2
matplotlib == 3.5.1
tomli == 2.0.1
REQ

zip -o -r "tmp/submissions/$DATE.zip" requirements.txt
rm requirements.txt

# Run and install scripts needed for CodaLab
cat > install.sh <<- INSTALL
#!/bin/bash

# Python dependencies
pip install -r requirements.txt
python -OO -m compileall .

# C++ code
mkdir -p release
cmake -Brelease -Shgs_vrptw -DCMAKE_BUILD_TYPE=Release
make --directory=release
INSTALL

cat > run.sh <<- RUN
#!/bin/bash

python solver.py
RUN

zip -o -r "tmp/submissions/$DATE.zip" run.sh install.sh
rm install.sh
rm run.sh

echo "Created tmp/submissions/$DATE.zip"
