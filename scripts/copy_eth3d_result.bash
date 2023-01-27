
hostname=$1
date=$2
ip=${hostname}.engin.umich.edu
    #for seq in sfm_lab_room_1
    for seq in  sfm_lab_room_1 ceiling_1 planar_2 sfm_garden repetitive sfm_house_loop
    do
	    folder=eth3d_${seq}_${date}
	    echo "folder ${folder}"
	    rsync -azP rzh@${ip}:~/code/docker_home/cvo/dsm/${folder}/ ${hostname}_results/${folder}

    done

