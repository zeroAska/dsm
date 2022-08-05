cd build && \
make -j && \
cd ..  
#gdb --args \
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg3_structure_notexture_near Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg1_360 Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
    
    #for seq in  ceiling_1 mannequin_3 sfm_bench sfm_lab_room_1 einstein_2 planar_2 #sfm_house_loop #360
    #for seq in  sfm_lab_room_1 einstein_2 planar_2 #sfm_house_loop #360
    for seq in  sfm_lab_room_1  #sfm_house_loop #360
    do
        rm -rf eth3d_${seq}
        mkdir -p eth3d_${seq}
        #gdb  --args 
	#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/${seq} Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 210 test_eth3d_${seq}.txt
	./build/bin/TumExample /home/rayzhang/media/Samsung_T5/eth3d/${seq} Examples/TumExample/cvo_rgbd_params_eth3d.yaml Examples/TumExample/eth3d_settings.txt 82 test_eth3d_${seq}.txt
        
        mv *.pcd eth3d_${seq}/
        mv *_graph.txt eth3d_${seq}/
        mv test_eth3d_${seq}.txt eth3d_${seq}/
        mv covisResult.txt eth3d_${seq}/
        mv log_eth3d_${seq}.txt eth3d_${seq}/
        sleep 5
        
    done
