#!/bin/bash

# This script formats all C++ source/header files in the 'common' and 'project' directories
# using clang-format (version 16 assumed).
# Only .cpp, .hpp, and .h files are processed.

set -e

CLANG_FORMAT=clang-format-16

# List of root folders to include
INCLUDE_DIRS=("common" "project")

echo "Running clang-format ..."

for DIR in "${INCLUDE_DIRS[@]}"; do
   if [ -d "$DIR" ]; then
      find "$DIR" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) \
         -exec "$CLANG_FORMAT" -i {} +
   else
      echo "Warning: directory '$DIR' does not exist, skipping."
   fi
done

echo "Formatting complete."
