cd build && \
make -j && \
cd ..  
#gdb --args \
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg3_structure_notexture_near Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg1_360 Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
    
    #for seq in  ceiling_1 mannequin_3 sfm_bench sfm_lab_room_1 einstein_2 planar_2 #sfm_house_loop #360
    for seq in  sfm_lab_room_1 einstein_2 planar_2 #sfm_house_loop #360
    do
        #gdb  --args 
	#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/${seq} Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 210 test_eth3d_${seq}.txt
        evo_traj tum ../eth3d_${seq}/test_eth3d_${seq}.txt -p --plot_mode xyz 
        
    done
