: ${1?"Usage: $0 repetitions"}
filename=benchmark_rosinit
i=0
while true
do if [ ! -f $filename$i.csv ]
then
	filename=$filename$i.csv
	break
fi
i=`echo "$i + 1" | bc`
done
echo "output will be written to $filename"
echo -n "benchmarking, please wait..."
echo "init[us],nodeHandle[us],param[us]" >> $filename
for i in `seq 1 $1`
	do rosrun rosinit rosinit _testparam=true >> $filename
done
echo "done"
