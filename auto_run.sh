#!/bin/bash
echo "+ Training time(mins): $1"
echo "# backup model#####"
deepracer_dir=./deepracer
models_dir=./deepracer/models
deepracer_on_the_spot_dir=./deepracer-on-the-spot
backup_dir=./deepracer/backup/
base=mariox-base
echo "pull latest code"
cd $deepracer_dir && git pull origin main
cd ..
cp -r $models_dir/* $backup_dir
j=0
for i in $(ls $models_dir);
do
	# if [ $j -gt 0 ]; then
	# 	echo "+ executing another [$j] tasks/task, before this, sleep for 1 mins"
	# 	sleep 60
	# fi
	echo "# copy files from $models_dir/$i/* to $deepracer_on_the_spot_dir/custom-files folder"
	cp -r $models_dir/$i/* $deepracer_on_the_spot_dir/custom-files/
	cd $deepracer_on_the_spot_dir
	sh ./create-spot-instance.sh $base $i $1
	#sh ./create-standard-instance.sh $base $i $1
	cd ../
	((j++))
done
echo "Completed"