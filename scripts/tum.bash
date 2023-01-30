date=$1
cd build && \
make -j && \
cd .. && \ 
#gdb --args \
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg3_structure_notexture_near Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
#./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg1_360 Examples/TumExample/cvo_rgbd_params.yaml Examples/TumExample/settings.txt 0 test_tum.txt
for idx in 1 #3
do
    
    for seq in desk2 desk #360
    do
	    rm *.pcd
	    rm *_graph.txt
	    folder=tum_freiburg${idx}_${seq}_${date}
        rm -rf ${folder}
        mkdir -p ${folder}
        rm *_graph.txt
        rm *.pcd
	#gdb --args \
		./build/bin/TumExample /home/rayzhang/media/Samsung_T5/tum/freiburg${idx}_${seq} Examples/TumExample/cvo_rgbd_params_eth3d.yaml Examples/TumExample/eth3d_settings_bki.txt 0 test_${folder}.txt
        
        mv *.pcd ${folder}/
        mv *_graph.txt ${folder}/
        mv test_${folder}.txt ${folder}/rkhs_slam.txt
        mv covisResult.txt ${folder}/
        mv log_${folder}.txt ${folder}/
	cp Examples/TumExample/cvo_rgbd_params_eth3d.yaml ${folder}/
	cp Examples/TumExample/eth3d_settings_bki.txt ${folder}/
        #sleep 5
        
    done
done
