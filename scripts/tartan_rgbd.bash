clear
cd build && \
make -j && \
cd .. && \ 
# gdb -ex r --args \
# gdb --args \
./build/bin/TartanRGBDExample /home/tannerliu/datasets/Tartan/carwelding/P001 Examples/TartanExample/RGB-D/cvo_rgbd_params.yaml Examples/TartanExample/RGB-D/settings.txt 0 test_tartan.txt
