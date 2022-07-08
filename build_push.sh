ros_versions=( "foxy" "galactic" "humble" "rolling")

for i in $(seq 1 4);
do
  ROS_DISTRO="${ros_versions[i]}"
  gcloud builds submit --config cloudbuild.yaml . --substitutions=_ROS_DISTRO="$ROS_DISTRO" --timeout=86400 &
  echo Dispached ROS "$ROS_DISTRO" using shaderobotics/pytorch:"$ROS_DISTRO" base image
done