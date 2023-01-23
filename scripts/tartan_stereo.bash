clear
cd build && \
make -j && \
cd .. && \ 

for difficulty in Easy
do
#for i in westerndesert  #hospital abandonedfactory_night #endofworld neighborhood oldtown  
#for i in  gascola soulcity abandonedfactory  seasidetown  seasonsforest seasonsforest_winter hospital endofworld neighborhood oldtown neighborhood  
for i in   gascola seasonsforest_winter seasonsforest soulcity 
	 do
	 rm -rf tartan_stereo_${difficulty}_$i
	mkdir -p tartan_stereo_${difficulty}_$i
./build/bin/TartanStereoExample_no_qt /home/rzh/media/sdg1/rzh/tartanair/$i/${difficulty}/P001 Examples/TartanExample/cvo_outdoor_params.yaml Examples/TartanExample/no_qt_settings.txt 0 test_stereo_${difficulty}_$i.txt #> log_tartan_${difficulty}_${i}.txt
mv *.pcd tartan_stereo_${difficulty}_$i/
mv *_graph.txt tartan_stereo_${difficulty}_$i/
cp test_stereo_${difficulty}_$i.txt /home/rzh/media/sdg1/rzh/tartanair/$i/${difficulty}/P001/rkhs_slam.txt
mv test_stereo_${difficulty}_$i.txt tartan_stereo_${difficulty}_$i/
mv covisResult.txt tartan_stereo_${difficulty}_$i/
mv log_tartan_stereo_${difficulty}_${i}.txt tartan_stereo_${difficulty}_$i/
sleep 5
done
done

