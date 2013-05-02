: ${2?"Usage: $0 /path/to/image.jpg repetitions"}
echo
filename=results
i=0
while true
do if [ ! -f $filename$i.csv ]
then
	filename=$filename$i.csv
	break
fi
i=`echo "$i + 1" | bc`
done
echo "results will be saved to $filename"
echo
echo -n "starting SIFT/SIFT ... "
rosrun feature_detection feature_detection_node _detector_type:=SIFT _extractor_type:=SIFT _image_path:=$1 _repetitions:=$2 _verbosity:=true >> $filename
echo "done"
echo -n "starting SURF/SURF ... "
rosrun feature_detection feature_detection_node _detector_type:=SURF _extractor_type:=SURF _image_path:=$1 _repetitions:=$2 _verbosity:=false >> $filename
echo "done"
echo -n "starting BRISK/BRISK ... "
rosrun feature_detection feature_detection_node _detector_type:=BRISK _extractor_type:=BRISK _image_path:=$1 _repetitions:=$2 _verbosity:=false >> $filename
echo "done"
echo -n "starting FAST/FREAK ... "
rosrun feature_detection feature_detection_node _detector_type:=FAST _extractor_type:=FREAK _image_path:=$1 _repetitions:=$2 _verbosity:=false >> $filename
echo "done"
echo -n "starting GFTT/BRIEF ... "
rosrun feature_detection feature_detection_node _detector_type:=GFTT _extractor_type:=BRIEF _image_path:=$1 _repetitions:=$2 _verbosity:=false >> $filename
echo "done"
echo -n "starting ORB/ORB ... "
rosrun feature_detection feature_detection_node _detector_type:=ORB _extractor_type:=ORB _image_path:=$1 _repetitions:=$2 _verbosity:=false >> $filename
echo "done"
echo
