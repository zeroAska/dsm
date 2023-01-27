
date=$1
cd build && \
make -j && \
cd ..  
#gdb --args \
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg3_structure_notexture_near Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg1_360 Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
    for seq in sfm_lab_room_1
    #for seq in  ceiling_1 planar_2 sfm_garden  sfm_lab_room_1 repetitive sfm_house_loop
    do
	    folder=eth3d_$seq_$date
        rm -rf $folder
        mkdir -p $folder
        #gdb -ex run --args 
	./build/bin/TumExample /home/rayzhang/media/Samsung_T5/eth3d/${seq} Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/eth3d_settings_raymap.txt  60 rkhs_slam.txt
	#./build/bin/TumExample_no_qt /home/rayzhang/media/Samsung_T5/eth3d/${seq} Examples/TumExample/cvo_rgbd_params_eth3d.yaml Examples/TumExample/eth3d_settings.txt 0 test_eth3d_${seq}.txt
	#./build/bin/TumExample /home/rzh/media/sdg1/rzh/eth3d/${seq} Examples/TumExample/cvo_rgbd_params_eth3d.yaml Examples/TumExample/eth3d_settings_raymap.txt 0 /home/rzh/media/sdg1/rzh/eth3d/${seq}/rkhs_slam.txt
        
        mv *.pcd $folder
        mv *_graph.txt $folder
        mv rkhs_slam.txt $folder
        mv covisResult.txt $folder
        mv log_eth3d_${seq}.txt $folder
        sleep 2

    done

