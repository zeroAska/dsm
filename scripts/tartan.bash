clear
cd build && \
make -j && \
cd .. && \ 
# gdb -ex r --args \
#	./build/bin/TartanExample /home/rayzhang/media/Samsung_T5/tartanair/office/Easy/P001 Examples/TartanExample/cvo_indoor_params.yaml Examples/TartanExample/indoor_settings.txt 290 test_easy_$i.txt
#	./build/bin/TartanExample /home/rayzhang/media/Samsung_T5/tartanair/abandonedfactory/Easy/P001 Examples/TartanExample/cvo_rgbd_params.yaml Examples/TartanExample/settings.txt 290 test_easy_$i.txt
for i in abandonedfactory abandonedfactory_night seasonsforest seasonsforest_winter
	 do
	 rm -rf tartan_easy_$i
	mkdir -p tartan_easy_$i
# gdb -ex r --args \
./build/bin/TartanRGBDExample /home/rayzhang/media/Samsung_T5/tartanair/$i/Easy/P001 Examples/TartanExample/cvo_outdoor_params.yaml Examples/TartanExample/outdoor_settings.txt 0 test_easy_$i.txt
mv *.pcd tartan_easy_$i/
mv *_graph.txt tartan_easy_$i/
mv test_easy_$i.txt tartan_easy_$i/
mv covisResult.txt tartan_easy_$i/
sleep 5
done

#for i in office hospital carwelding
#do
#    rm -rf tartan_easy_$i
#    mkdir -p tartan_easy_$i
    
#./build/bin/TartanExample /home/rayzhang/media/Samsung_T5/tartanair/$i/Easy/P001 Examples/TartanExample/cvo_indoor_params.yaml Examples/TartanExample/indoor_settings.txt 0 test_easy_$i.txt
#mv *.pcd tartan_easy_$i/
#mv *_graph.txt tartan_easy_$i/
#mv test_easy_$i.txt tartan_easy_$i/
#mv covisResult.txt tartan_easy_$i/
#done

