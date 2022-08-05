seq=$1


cd build
make -j
cd ..
#gdb -ex run --args \
gdb --args \
./build/bin/TumExample_depth_test /home/rayzhang/media/Samsung_T5/tum/sfm_house_loop  Examples/TumExample/settings_depth.txt /home/rayzhang/media/Samsung_T5/tum/sfm_house_loop/groundtruth_aligned.txt Examples/TumExample/eth3d_depth_input.txt Examples/TumExample/cvo_rgbd_params.yaml
