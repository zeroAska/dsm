
clear
export CUDA_VISIBLE_DEVICES=0
cd build 
make -j 
cd ..  
#for i in westerndesert  #hospital abandonedfactory_night #endofworld neighborhood oldtown 
for label in color
do
#for i in  gascola soulcity abandonedfactory  seasidetown  seasonsforest seasonsforest_winter hospital endofworld neighborhood oldtown neighborhood  
for i in 03 #07 05    #soulcity  seasidetown  hospital   
	 do
	 rm -rf kitti_${label}_$i
	mkdir -p kitti_${label}_$i
#gdb -ex run  --args \
./build/bin/KittiExample_NO_QT /home/rzh/media/sda1/ray/datasets/kitti/sequences/$i Examples/KittiExample/cvo_stereo_params.yaml Examples/KittiExample/settings.txt 0 test_${label}_${i}.txt
#./build/bin/TartanRGBDExample /home/rayzhang/media/Samsung_T5/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_outdoor_params.yaml Examples/TartanExample/outdoor_settings.txt 0 test_${difficulty}_$i.txt > log_tartan_${difficulty}_${i}.txt
mv *.pcd kitti_${label}_$i/
mv *_graph.txt kitti_${label}_$i/
mv test_${label}_$i.txt kitti_${label}_$i/
mv covisResult.txt kitti_${label}_$i/
mv log_kitti_${label}_${i}.txt kitti_${label}_$i/
sleep 5
done
done
#for i in hospital  carwelding
#do
#    rm -rf tartan_Easy_$i
#    mkdir -p tartan_Easy_$i
    
#./build/bin/TartanExample /home/rayzhang/media/Samsung_T5/tartanair/$i/Easy/P001 Examples/TartanExample/cvo_indoor_params.yaml Examples/TartanExample/indoor_settings.txt 0 test_Easy_$i.txt
#mv *.pcd tartan_Easy_$i/
#mv *_graph.txt tartan_Easy_$i/
#mv test_Easy_$i.txt tartan_Easy_$i/
#mv covisResult.txt tartan_Easy_$i/
#done
##
