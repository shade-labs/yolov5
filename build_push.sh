ros_versions=( "foxy" "galactic" "humble" "rolling")
cuda_versions=( "foxy-cudnn" "galactic-cudnn" "humble-cuda" "rolling-cuda")

for i in $(seq 1 4);
do
  ROS_DISTRO="${ros_versions[i]}"
  CUDA_VERSION="${cuda_versions[i]}"
  gcloud builds submit --config cloudbuild.yaml . --substitutions=_ROS_DISTRO="$ROS_DISTRO",_CUDA_VERSION="$CUDA_VERSION" --timeout=86400 &
  echo Dispached ROS "$ROS_DISTRO" using shaderobotics/ros:"$CUDA_VERSION" base image
done