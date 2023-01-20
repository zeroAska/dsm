cd build && \
make -j && \
cd ..  
#gdb --args \
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg3_structure_notexture_near Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg1_360 Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
    
    for seq in  ceiling_1 planar_2 sfm_garden  sfm_lab_room_1 repetitive sfm_house_loop
    do
        rm -rf eth3d_${seq}
        mkdir -p eth3d_${seq}
        #gdb  --args 
	#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/${seq} Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 210 test_eth3d_${seq}.txt
	./build/bin/TumExample_no_qt /home/rzh/media/sdg1/rzh/eth3d/${seq} Examples/TumExample/cvo_rgbd_params_eth3d.yaml Examples/TumExample/eth3d_settings.txt 0 /home/rzh/media/sdg1/rzh/eth3d/${seq}/rkhs_slam.txt
        
        mv *.pcd eth3d_${seq}/
        mv *_graph.txt eth3d_${seq}/
        mv test_eth3d_${seq}.txt eth3d_${seq}/
        mv covisResult.txt eth3d_${seq}/
        mv log_eth3d_${seq}.txt eth3d_${seq}/
        sleep 5

    done

