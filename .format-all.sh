# /bin/bash

exclude_list="Util bush"
all_files=$(git ls-files *.h *.hpp *.c *.cpp)

for exclude in $exclude_list
do
    all_files=$(echo "$all_files" | sed "/$exclude/d")
done

echo "$all_files"
clang-format-10 -i ${all_files}