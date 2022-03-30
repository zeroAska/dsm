export CUDA_VISIBLE_DEVICES=1
clear
#cd build && make -j && cd ..

for difficulty in Easy #Hard
do
for i in abandonedfactory #abandonedfactory_night seasonsforest seasonsforest_winter seasidetown gascola
do
	rm -rf tartan_${difficulty}_$i
	mkdir -p tartan_${difficulty}_$i
            echo " Current Seq: ${i} ${difficulty}"
        # gdb -ex r --args \
./build/bin/TartanRGBDExample /home/tannerliu/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_outdoor_params.yaml Examples/TartanExample/outdoor_settings.txt 0 test_${difficulty}_$i.txt #> log_tartan_${difficulty}_${i}.txt
mv *.pcd tartan_${difficulty}_$i/
mv *_graph.txt tartan_${difficulty}_$i/
mv test_${difficulty}_$i.txt tartan_${difficulty}_$i/
mv covisResult.txt tartan_${difficulty}_$i/
mv log_tartan_${difficulty}_${i}.txt tartan_${difficulty}_$i/
sleep 5
    
#gdb -ex r --args \
#    ./build/bin/TartanSemanticExample /home/tannerliu/media/Samsung_T5/tartanair/abandonedfactory/Easy/P001 Examples/TartanExample/Semantic/cvo_rgbd_params.yaml Examples/TartanExample/Semantic/settings.txt 0 test_tartan.txt

done
done