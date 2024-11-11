
date=$1
clear
#cd build && make -j && cd ..
#cd build_debug && make -j && cd ..

for difficulty in Easy #Hard
do
	for i in abandonedfactory gascola hospital seasidetown seasonsforest seasonsforest_winter soulcity
#for i in abandonedfactory #seasidetown #endofworld gascola  soulcity ocean
do
	folder=tartan_semantic_${difficulty}_${i}_${date}
	 rm -rf $folder
	mkdir -p $folder
            echo " Current Seq: ${i} ${difficulty}"
#         gdb -ex r --args \
./build/bin/TartanSemanticExample_no_qt /home/rayzhang/media/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_semantic_params.yaml Examples/TartanExample/no_qt_semantic_settings.txt 0 test_semantic_${difficulty}_$i.txt #> log_tartan_semantic_${difficulty}_${i}.txt
mv *.pcd $folder #tartan_semantic_${difficulty}_$i/
mv *_graph.txt $folder #tartan_semantic_${difficulty}_$i/
mv test_semantic_${difficulty}_$i.txt $folder  #tartan_semantic_${difficulty}_$i/
cp Examples/TartanExample/cvo_semantic_params.yaml $folder/
cp Examples/TartanExample/no_qt_semantic_settings.txt $folder/
mv covisResult.txt $folder #tartan_semantic_${difficulty}_$i/
mv log_tartan_semantic_${difficulty}_${i}.txt $folder #tartan_semantic_${difficulty}_$i/
sleep 5
    
#gdb -ex r --args \
#    ./build/bin/TartanSemanticExample /home/tannerliu/media/Samsung_T5/tartanair/abandonedfactory/Easy/P001 Examples/TartanExample/Semantic/cvo_rgbd_params.yaml Examples/TartanExample/Semantic/settings.txt 0 test_tartan.txt

done
done

