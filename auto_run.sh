#!/bin/bash
echo "+ Training time(mins): $1"
echo "# backup model#####"
cp -r ./deepracer/models/* ./deepracer/backup/
j=0
for i in $(ls ../deepracer/models);
do
	echo "# copy files ./deepracer/models/$i/* to ./deepracer-on-the-spot/custom-files folder"
	cp -r ./deepracer/models/$i/* ./deepracer-on-the-spot/custom-files/
	cd ./deepracer-on-the-spot/
	sh ./create-spot-instance.shbrew install jq mariox-base $i $1
	#./create-standard-instance.sh mariox-base $i $1
	if [ $j -gt 0 ]; then
		echo "+ sleep for 5 mins"
		sleep 600
	fi
	((j++))
done
echo "Completed"