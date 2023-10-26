cd build
make -j
cd ..
date=$1

clear
#cd build && make -j && cd ..

for difficulty in Easy #Hard
do
	for i in abandonedfactory gascola hospital seasidetown seasonsforest seasonsforest_winter soulcity
do
    folder=tartan_rgbd_voxelmap_${difficulty}_${i}_$date
	 rm -rf $folder
	mkdir -p $folder
        # gdb -ex r --args \
            echo " Current Seq: ${i} ${difficulty}"
      #      gdb -ex r --args \
./build/bin/TartanRGBDExample /home/rayzhang/media/Samsung_T5/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_outdoor_params.yaml Examples/TartanExample/outdoor_settings.txt 0 test_rgbd_${difficulty}_$i.txt #> log_tartan_rgbd_${difficulty}_${i}.txt
mv *.pcd $folder
mv *_graph.txt $folder #tartan_semantic_${difficulty}_$i/
mv test_rgbd_${difficulty}_$i.txt $folder/rkhs_slam.txt  #tartan_semantic_${difficulty}_$i/
mv covisResult.txt $folder #tartan_semantic_${difficulty}_$i/
mv log_tartan_rgbd_${difficulty}_${i}.txt $folder
sleep 5
    
#gdb -ex r --args \
#    ./build/bin/TartanSemanticExample /home/tannerliu/media/Samsung_T5/tartanair/abandonedfactory/Easy/P001 Examples/TartanExample/Semantic/cvo_rgbd_params.yaml Examples/TartanExample/Semantic/settings.txt 0 test_tartan.txt

done
done
