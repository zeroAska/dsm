seq=$1


cd build
make -j
cd ..
#gdb -ex run --args \
gdb --args \
./build/bin/KittiExample_depth_filter /home/rzh/media/sda1/ray/datasets/kitti/sequences/$seq Examples/KittiExample/settings_depth.txt /home/rzh/unified_cvo/results/cvo_intensity_img_gpu0_oct25_best/${seq}.txt Examples/KittiExample/depth_test_input.txt Examples/KittiExample/cvo_stereo_params.yaml
