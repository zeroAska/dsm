cd build && \
make -j && \
cd ..  
#gdb --args \
./build/bin/KittiExample /home/rayzhang/media/Samsung_T5/kitti/05 Examples/KittiExample/cvo_stereo_params.yaml Examples/KittiExample/settings.txt 0 test_kitti.txt
