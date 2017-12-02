for i in *.pcd; do
	echo "$i"
	echo "size 0.1"
	#correr 10 veces
	for run in {1..10}
	do
	    ./corruptor "$i" 0.1
	done
	echo "size 0.5"
	#correr 10 veces
	for run in {1..10}
	do
	    ./corruptor "$i" 0.5
	done
	echo "size 1"
	#correr 10 veces
	for run in {1..10}
	do
	    ./corruptor "$i" 1
	done
done
