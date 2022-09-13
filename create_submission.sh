#!/bin/bash

DATE=$(date "+%Y-%m-%d-%H-%M-%S")
mkdir -p tmp/submissions

# C++ source and header files (incl. pybind)
find hgs_vrptw \( -name "*.cpp" -o -name "*.h" -o -name "CMakeLists.txt" -o -name "*.cmake" \) -exec zip -o -r "tmp/submissions/$DATE.zip" {} +

# Python dependencies need to be provided as a requirements.txt, since we can
# only use pip in the competition system.
cat > requirements.txt <<- REQ
numpy == 1.22.2
matplotlib == 3.5.1
REQ

zip -o -r "tmp/submissions/$DATE.zip" requirements.txt
rm requirements.txt

# Python code and install/run scripts
zip -o -r "tmp/submissions/$DATE.zip" solver.py tools.py strategies.py environment.py
zip -o -r "tmp/submissions/$DATE.zip" metadata run.sh install.sh

echo "Created submissions/submission_$DATE.zip"
