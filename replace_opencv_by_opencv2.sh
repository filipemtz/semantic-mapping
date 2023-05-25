#!/bin/bash

# Find all files ending with .h, .c, or .cpp in the current directory and its subdirectories
files=$(find . -type f \( -name "*.h" -o -name "*.c" -o -name "*.cpp" \))

# Replace all occurrences of "#include <opencv/[something]>" with "#include <opencv2/[something]>"
for file in $files; do
  sed -i 's/libcarmen_util/libsegmap\/libcarmen_util/g' $file
done
