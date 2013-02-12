: ${1?"Usage: $0 /path/to/image.jpg"}
filename=benchmark_feature_detection
i=0
verbose=true;
while true
do if [ ! -f $filename$i.csv ]
then
	filename=$filename$i.csv
	break
fi
i=`echo "$i + 1" | bc`
done
for j in SURF BRISK ORB FAST SIFT
	do for i in SURF SIFT BRISK FREAK ORB BRIEF
		do rosrun feature_detection feature_detection_node _detector_type:=$j _extractor_type:=$i _image_path:=$1 _repetitions:=1 _verbosity:=$verbose >> $filename
		verbose=false;
	done
done
