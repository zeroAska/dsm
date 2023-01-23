cd build && \
make -j && \
cd ..  
#gdb --args \
./build/bin/KittiExample /home/rayzhang/media/Samsung_T5/kitti_stereo/dataset/sequences/kitti/03 Examples/TartanExample/cvo_outdoor_params.yaml Examples/KittiExample/settings.txt 0 test_kitti.txt
