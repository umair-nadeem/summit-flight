#!/bin/bash

set -e

# Project root relative to this script
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

for toolchain in stm_m4 native
do
   compile_commands="$project_root/out/build/$toolchain/compile_commands.json"

   if [[ -f $compile_commands ]]; then
      cppcheck \
      --enable=all \
      --inconclusive \
      --std=c++20 \
      --language=c++ \
      --inline-suppr \
      --suppress=unusedFunction \
      --suppress=missingInclude \
      --suppress=*:*\/os\* \
      -i $project_root/external \
      -i $project_root/platform \
      --quiet \
      --error-exitcode=1 \
      --project=$compile_commands
   fi
done
