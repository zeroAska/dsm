clear

date=$1
cd build && \
make -j && \
cd .. && \ 
for difficulty in Easy
do
#for i in westerndesert  #hospital abandonedfactory_night #endofworld neighborhood oldtown  
#for i in  gascola soulcity abandonedfactory  seasidetown  seasonsforest seasonsforest_winter hospital endofworld  

for i in abandonedfactory  gascola  hospital  seasidetown  seasonsforest  seasonsforest_winter  soulcity	
#for i in  hospital #seasidetown  soulcity
	 do
		 folder=tartan_${difficulty}_${i}_$date
	 rm -rf tartan_${difficulty}_${i}_$date
	mkdir -p tartan_${difficulty}_${i}_${date}
#gdb -ex run  --args \
#./build/bin/TartanRGBDExample /home/rayzhang/media/Samsung_T5/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_outdoor_params.yaml Examples/TartanExample/outdoor_settings.txt 0 test_${difficulty}_$i.txt #> log_tartan_${difficulty}_${i}.txt
./build/bin/TartanRGBDExample_no_qt /home/rzh/media/sdc1/rzh/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_outdoor_params.yaml Examples/TartanExample/no_qt_settings.txt 0 test_${difficulty}_$i.txt #> log_tartan_${difficulty}_${i}.txt
mv *.pcd $folder
mv *_graph.txt $folder
mv test_${difficulty}_$i.txt $folder/
mv covisResult.txt $folder
mv log_tartan_${difficulty}_${i}.txt $folder
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
