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
echo "init;nodeHandle;param" >> $filename
for i in `seq 1 10`
	do rosrun image_benchmark rosinit_benchmark_node _testparam=true >> $filename
done
