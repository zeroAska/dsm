cd build
make -j
cd ..
date=$1

clear
#cd build && make -j && cd ..

for difficulty in Easy #Hard
do
for i in abandonedfactory abandonedfactory_night seasonsforest seasonsforest_winter gascola seasidetown 
	#endofworld gascola  soulcity ocean
do
    folder=tartan_rgbd_${difficulty}_${i}_$date
	 rm -rf $folder
	mkdir -p $folder
        # gdb -ex r --args \
            echo " Current Seq: ${i} ${difficulty}"
            #gdb -ex r --args \
./build/bin/TartanRGBDExample /home/rayzhang/media/Samsung_T5/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_rkhs_params.yaml Examples/TartanExample/rkhs_settings.txt 0 test_rgbd_${difficulty}_$i.txt 1 	#> log_tartan_rgbd_${difficulty}_${i}.txt
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
