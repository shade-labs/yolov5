ros_versions=( "foxy" "galactic" "humble" "rolling")

for ROS_DISTRO in "${ros_versions[@]}"
do
  gcloud builds submit --config cloudbuild.yaml . --substitutions=_ROS_DISTRO="$ROS_DISTRO" --timeout=86400 &
  echo Dispached ROS "$ROS_DISTRO" using shaderobotics/yolov5:"$ROS_DISTRO" base image
done