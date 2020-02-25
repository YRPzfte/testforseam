#! /bin/bash
rm *png
rm *jpg
# 保存当前目录
currentDir=$PWD
echo "Start to publish...\n"
git add --all
time=$(date "+%Y%m%d-%H%M%S")
git commit -m $time
# 执行git命令
git push
echo "Success\n";