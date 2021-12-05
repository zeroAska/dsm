cd build && \
make -j && \
cd .. && \ 
gdb -ex r --args \
./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg1_desk2/ Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
