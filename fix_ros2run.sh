#!/bin/bash
# colcon build 后执行, 修复 ros2 run 找不到可执行文件
PREFIX=~/vb300_ws/install/vb300_nodes
mkdir -p $PREFIX/lib/vb300_nodes
for f in $PREFIX/bin/*; do
  name=$(basename "$f")
  ln -sf "../../bin/$name" "$PREFIX/lib/vb300_nodes/$name" 2>/dev/null
done
echo "✅ ros2 run 修复完成"
