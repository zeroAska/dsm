
hostname=$1
date=$2
ip=${hostname}.engin.umich.edu
#for seq in sfm_lab_room_1
mkdir -p ${hostname}_results
    for seq in desk desk2 room

    do
	    folder=tum_freiburg1_${seq}_${date}
	    echo "folder ${folder}"
	    rsync -azP rzh@${ip}:~/code/docker_home/cvo/dsm2/dsm/${folder}/ ${hostname}_results/${folder}

    done

