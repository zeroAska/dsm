clear
cd build && \
make -j && \
cd .. && \ 
# gdb --args \
gdb -ex r --args \
./build/bin/TartanSemanticExample /home/tannerliu/media/Samsung_T5/tartanair/abandonedfactory/Easy/P001 Examples/TartanExample/Semantic/cvo_rgbd_params.yaml Examples/TartanExample/Semantic/settings.txt 0 test_tartan.txt
