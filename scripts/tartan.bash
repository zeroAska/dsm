clear
cd build && \
make -j && \
cd .. && \ 
# gdb -ex r --args \
# gdb --args \
./build/bin/TartanExample /home/tannerliu/datasets/Tartan/P001 Examples/TartanExample/cvo_rgbd_params.yaml Examples/TartanExample/settings.txt 0 test_tartan.txt
