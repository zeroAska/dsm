clear
cd build && \
make -j && \
cd .. && \ 
# gdb -ex r --args \
# gdb --args \
for i in carwelding
do
#for i in hospital carwelding abandonedfactory amusement
./build/bin/TartanExample /home/rayzhang/media/Samsung_T5/tartanair/$i/Easy/P001 Examples/TartanExample/cvo_rgbd_params.yaml Examples/TartanExample/settings.txt 0 test_tartan.txt
done
