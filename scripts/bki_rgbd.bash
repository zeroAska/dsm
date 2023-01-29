cd build &&
make -j &&
cd .. &&
gdb -ex run --args \
./build/bin/TumExample_local_mapping /home/rayzhang/media/Samsung_T5/tum/freiburg1_desk2 Examples/TumExample/cvo_rgbd_params_eth3d.yaml 0 40 /home/rayzhang/media/Samsung_T5/tum/freiburg1_desk2/A-CVO.txt 1
