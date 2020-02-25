#!/usr/bin/env bash
reset
if [ ! -d build  ];then
  mkdir build
fi
source_dir="datasets/会议室/save_1"
target_dir="datasets/会议室/save_2"
cd build
cmake ..
make
cd ..
#./build/zhang_15_stitching $source_dir $target_dir
python detect_SIFT.py --source-dir=$source_dir --target-dir=$target_dir
./build/stitch_based_on_3D $source_dir $target_dir