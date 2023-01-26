cd build
make -j
cd ..
#gdb -ex run --args \

### for eth3d example
#./build/bin/TumExample_local_mapping /home/rayzhang/media/Samsung_T5/eth3d/sfm_lab_room_1 cvo_params/cvo_intensity_params_irls_eth3d.yaml 0 50 /home/rayzhang/media/Samsung_T5/eth3d/sfm_lab_room_1/CVO.txt

### for tum example
#gdb -ex run --args \
./build/bin/TartanRGBDExample_local_mapping /home/rayzhang/media/Samsung_T5/tartanair/abandonedfactory/Easy/P001 cvo_params/cvo_intensity_params_irls_eth3d.yaml 0 200 /home/rayzhang/unified_cvo/cvo_tartan_outdoor_abandonedfactory.txt 19
