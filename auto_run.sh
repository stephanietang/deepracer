#!/bin/bash
echo "+ Training time(mins): $1"
echo "# backup model#####"
models_dir=./deepracer/models
deepracer_on_the_spot_dir=./deepracer-on-the-spot
backup_dir=./deepracer/backup/
base=mariox-base
cp -r $models_dir/* $backup_dir
j=0
for i in $(ls $models_dir);
do
	echo "# copy files from $models_dir/$i/* to $deepracer_on_the_spot_dir/custom-files folder"
	cp -r $models_dir/$i/* $deepracer_on_the_spot_dir/custom-files/
	cd $deepracer_on_the_spot_dir
	sh ./create-spot-instance.sh $base $i $1
	#./create-standard-instance.sh $base $i $1
	if [ $j -gt 0 ]; then
		echo "+ sleep for 5 mins"
		sleep 600
	fi
	((j++))
done
echo "Completed"