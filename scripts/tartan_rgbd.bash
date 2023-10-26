cd build
make -j
cd ..
date=$1

clear
<<<<<<< HEAD
date=$1
cd build && \
make -j && \
cd .. && \ 
for difficulty in Easy
do
for i in abandonedfactory  gascola  hospital  seasidetown  seasonsforest  seasonsforest_winter  soulcity
do
    rm *.pcd
    rm *_graph.txt
    folder=tartan_${difficulty}_${i}_$date
    rm -rf $folder
    mkdir -p $folder

#gdb -ex run  --args \
#./build/bin/TartanRGBDExample /home/rayzhang/media/Samsung_T5/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_outdoor_params.yaml Examples/TartanExample/outdoor_settings.txt 0 test_${difficulty}_$i.txt #> log_tartan_${difficulty}_${i}.txt
#gdb -ex run  --args \
./build/bin/TartanRGBDExample_no_qt /home/rzh/media/sdc1/rzh/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_outdoor_params.yaml Examples/TartanExample/no_qt_semantic_settings.txt 0 tartan_${difficulty}_$i.txt  > log_tartan_${difficulty}_${i}.txt
mv *.pcd $folder
mv *_graph.txt $folder
mv tartan_${difficulty}_${i}.txt $folder/rkhs_slam.txt
mv covisResult.txt $folder
mv log_tartan_${difficulty}_${i}.txt $folder
sleep 2
=======
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

>>>>>>> 5caa23044a46024820332a87fdc0917db9c8710c
done
done
