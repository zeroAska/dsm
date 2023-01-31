clear
cd build && \
make -j && \
cd .. && \ 
# gdb -ex r --args \
# gdb --args \
./build/bin/TartanRGBDExample /home/rayzhang/media/Samsung_T5/tartanair/abandonedfactory/Easy/P001 Examples/TartanExample/cvo_outdoor_params.yaml Examples/TartanExample/outdoor_settings.txt 0 test_tartan.txt
