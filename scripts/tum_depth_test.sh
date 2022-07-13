seq=$1


cd build
make -j
cd ..
#gdb -ex run --args \
gdb --args \
./build/bin/TumExample_depth_test /home/rzh/media/sda1/ray/datasets/tum/freiburg1_${seq}/  Examples/TumExample/settings_depth.txt /home/rzh/media/sda1/ray/datasets/tum/freiburg1_${seq}/A-CVO.txt Examples/TumExample/depth_test_input.txt Examples/TumExample/cvo_rgbd_params.yaml
