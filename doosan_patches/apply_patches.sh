#!/bin/bash
# Mini PC'de doosan-robot2 kurulduktan sonra çalıştır
# Kullanım: bash doosan_patches/apply_patches.sh

WORKSPACE=$(cd "$(dirname "$0")/.." && pwd)
DOOSAN="$WORKSPACE/src/doosan-robot2"

if [ ! -d "$DOOSAN" ]; then
    echo "HATA: $DOOSAN bulunamadı"
    echo "Önce doosan-robot2'yi src/ klasörüne klonla:"
    echo "  cd $WORKSPACE/src"
    echo "  git clone https://github.com/doosan-robotics/doosan-robot2.git"
    exit 1
fi

cp "$WORKSPACE/doosan_patches/dsr_bringup2/launch/dsr_bringup2_rviz.launch.py" \
   "$DOOSAN/dsr_bringup2/launch/"

cp "$WORKSPACE/doosan_patches/dsr_description2/rviz/default.rviz" \
   "$DOOSAN/dsr_description2/rviz/"

echo "Patch'ler uygulandı. Şimdi build et:"
echo "  cd $WORKSPACE && colcon build --symlink-install"
