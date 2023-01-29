cd build && \
make -j && \
cd .. && \ 
#gdb --args \
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg3_structure_notexture_near Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg1_360 Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
for idx in 1 #3
do
    
    for seq in  desk2 #360
    do
        rm -rf tum_fr${idx}_${seq}
        mkdir -p tum_fr${idx}_${seq}
        	
	gdb --args \
		./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg${idx}_${seq} Examples/TumExample/cvo_rgbd_params_eth3d.yaml Examples/TumExample/eth3d_settings_bki.txt 0 test_tum_fr${idx}_${seq}.txt
        
        mv *.pcd tum_fr${idx}_${seq}/
        mv *_graph.txt tum_fr${idx}_${seq}/
        mv test_tum_fr${idx}_${seq}.txt tum_fr${idx}_${seq}/
        mv covisResult.txt tum_fr${idx}_${seq}/
        mv log_tum_fr${idx}_${seq}.txt tum_fr${idx}_${seq}/
	cp Examples/TumExample/cvo_rgbd_params_eth3d.yaml tum_fr${idx}_${seq}/
	cp Examples/TumExample/eth3d_settings.txt tum_fr${idx}_${seq}/
        #sleep 5
        
    done
done
