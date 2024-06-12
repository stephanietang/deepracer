
echo "Training time(mins): $1"
echo "#######backup model"
cp -r ./models/* .deepracer/backup/
for i in $(ls ./models)
do
	echo "copy files ./models/$i/* to ./deepracer-on-the-spot/custom-files folder"
	cp -r ./models/$i/* ./deepracer-on-the-spot/custom-files/
	cd ./deepracer-on-the-spot/
	./create-spot-instance.sh steph-base $i $1
	#./create-standard-instance.sh steph-base $i $1
done
echo "Completed"
