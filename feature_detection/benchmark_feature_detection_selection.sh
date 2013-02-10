: ${1?"Usage: $0 /path/to/image.jpg"}
filename=benchmark_feature_detection_selection
i=0
while true
do if [ ! -f $filename$i.csv ]
then
	filename=$filename$i.csv
	break
fi
i=`echo "$i + 1" | bc`
done
rosrun image_benchmark feature_detection_node _detector_type:=SIFT _extractor_type:=SIFT _image_path:=$1 _repetitions:=10 _verbosity:=true >> $filename

rosrun image_benchmark feature_detection_node _detector_type:=SURF _extractor_type:=SURF _image_path:=$1 _repetitions:=10 _verbosity:=false >> $filename

rosrun image_benchmark feature_detection_node _detector_type:=BRISK _extractor_type:=BRISK _image_path:=$1 _repetitions:=10 _verbosity:=false >> $filename

rosrun image_benchmark feature_detection_node _detector_type:=FAST _extractor_type:=FREAK _image_path:=$1 _repetitions:=10 _verbosity:=false >> $filename

rosrun image_benchmark feature_detection_node _detector_type:=GFTT _extractor_type:=BRIEF _image_path:=$1 _repetitions:=10 _verbosity:=false >> $filename

rosrun image_benchmark feature_detection_node _detector_type:=ORB _extractor_type:=ORB _image_path:=$1 _repetitions:=10 _verbosity:=false >> $filename
