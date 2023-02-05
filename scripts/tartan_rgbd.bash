clear
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
done
done
