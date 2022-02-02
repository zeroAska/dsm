clear
cd build && \
make -j && \
cd .. && \ 
# gdb --args \
gdb -ex r --args \
./build/bin/TartanStereoExample /home/tannerliu/datasets/Tartan/abandonedfactory/P001 Examples/TartanExample/Stereo/cvo_stereo_params.yaml Examples/TartanExample/Stereo/settings.txt 0 test_tartan.txt
