cd build
make -j
cd ..

date=$1
clear
#cd build && make -j && cd ..
#for i in abandonedfactory #seasidetown #endofworld gascola  soulcity ocean
for difficulty in Easy #Hard
do
    skylabel=(196 112 -- 130  196 146 130)
    seqs=(abandonedfactory gascola hospital seasidetown seasonsforest seasonsforest_winter soulcity)
    #skylabel=(196 112 130  196 146 130)
    #seqs=(abandonedfactory gascola  seasidetown seasonsforest seasonsforest_winter soulcity)
    #skylabel=(130 196 112 130 146)
    #seqs=(seasidetown seasonsforest gascola soulcity seasonsforest_winter)
    for ind in ${!seqs[@]}
    do
        i=${seqs[ind]}
        sky=${skylabel[ind]}
	folder=tartan_semantic_${difficulty}_${i}_bki_${date}
        echo " Current Seq: ${i} ${difficulty} with sky label ${sky}"        
	rm -rf $folder
	mkdir -p $folder
	rm *.pcd
	rm *_graph.txt

#                gdb -ex run --args \
                    ./build/bin/TartanSemanticExample /home/rayzhang/media/Samsung_T5/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_semantic_params.yaml Examples/TartanExample/semantic_settings.txt 0 test_semantic_${difficulty}_$i.txt $sky  > log_tartan_semantic_${difficulty}_${i}.txt
                    mv *.pcd $folder/
                    mv *_graph.txt $folder
                    mv test_semantic_${difficulty}_$i.txt $folder/rkhs_slam.txt
                    mv covisResult.txt $folder
                    mv log_tartan_semantic_${difficulty}_${i}.txt $folder
                    sleep 2

    done
done

