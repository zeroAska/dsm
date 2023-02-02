cd build
make -j
cd ..

date=$1
clear
#cd build && make -j && cd ..
               #for i in abandonedfactory #seasidetown #endofworld gascola  soulcity ocean
for difficulty in Easy #Hard
do
    skylabel=(196 112 130 196 146 130)
    seqs=(abandonedfactory gascola hospital seasidetown seasonsforest seasonsforest_winter soulcity)
    for ind in ${!seqs[@]}
    do
        i=${seqs[ind]}
        sky=${skylabel[ind]}
	folder=tartan_semantic_${difficulty}_${i}_${date}
        echo " Current Seq: ${i} ${difficulty} with sky label ${sky}"        
	rm -rf $folder
	mkdir -p $folder

            #gdb --args \
                ./build/bin/TartanSemanticExample /home/rayzhang/media/Samsung_T5/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_semantic_params.yaml Examples/TartanExample/semantic_settings.txt 0 test_semantic_${difficulty}_$i.txt  log_tartan_semantic_${difficulty}_${i}.txt
                mv *.pcd $folder/
                mv *_graph.txt $folder
                mv test_semantic_${difficulty}_$i.txt $folder/rkhs_slam.txt
                mv covisResult.txt $folder
                mv log_tartan_semantic_${difficulty}_${i}.txt $folder
                sleep 2

    done
done

