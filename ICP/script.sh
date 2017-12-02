for i in *.obj; do
# iterar en size y error
#valores de size -> 0.01, 0.05, 0.1 (1cm, 5cm y 10cm)
#valores de error -> 0.02, 0.03, 0.05 (2cm, 3cm, 5cm)
#./interactive_icp bote.obj -v -i 0.05 0.0001

	echo "$i"
	echo "size 0.01 error 0.02"
	#correr 10 veces
	for run in {1..10}
	do
		./interactive_icp "$i" -v -i 0.01 0.02
	done
	echo "size 0.01 error 0.03"
	#correr 10 veces
	for run in {1..10}
	do
		./interactive_icp "$i" -v -i 0.01 0.03
	done
	echo "size 0.01 error 0.05"
	#correr 10 veces
	for run in {1..10}
	do
		./interactive_icp "$i" -v -i 0.01 0.05
	done
	echo "size 0.05 error 0.02"
	#correr 10 veces
	for run in {1..10}
	do
		./interactive_icp "$i" -v -i 0.05 0.02
	done
	echo "size 0.05 error 0.03"
	#correr 10 veces
	for run in {1..10}
	do
		./interactive_icp "$i" -v -i 0.05 0.03
	done
	echo "size 0.05 error 0.05"
	#correr 10 veces
	for run in {1..10}
	do
		./interactive_icp "$i" -v -i 0.05 0.05
	done

	echo "size 0.1 error 0.02"
	#correr 10 veces
	for run in {1..10}
	do
		./interactive_icp "$i" -v -i 0.1 0.02
	done
	echo "size 0.1 error 0.03"
	#correr 10 veces
	for run in {1..10}
	do
		./interactive_icp "$i" -v -i 0.1 0.03
	done
	echo "size 0.1 error 0.05"
	#correr 10 veces
	for run in {1..10}
	do
		./interactive_icp "$i" -v -i 0.1 0.05
	done


done
