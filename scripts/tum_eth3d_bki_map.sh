cd build
make -j
cd ..
#gdb -ex run --args \

### for eth3d example
#./build/bin/TumExample_local_mapping /home/rayzhang/media/Samsung_T5/eth3d/sfm_lab_room_1 cvo_params/cvo_intensity_params_irls_eth3d.yaml 0 50 /home/rayzhang/media/Samsung_T5/eth3d/sfm_lab_room_1/CVO.txt

### for tum example
#gdb -ex run --args \
#./build/bin/TumExample_local_mapping /home/rayzhang/media/Samsung_T5/tum/freiburg1_desk2 cvo_params/cvo_intensity_params_irls_eth3d.yaml 0 10 /home/rayzhang/media/Samsung_T5/tum/freiburg1_desk2/A-CVO.txt
