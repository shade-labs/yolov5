ros_versions=( "foxy" "galactic" "humble" "rolling")
ubuntu_versions=( "focal" "focal" "jammy" "jammy")
cuda_versions=( "foxy-cudnn" "galactic-cudnn" "humble-cuda" "rolling-cuda")

for i in $(seq 1 3);
do
  ROS_DISTRO="${ros_versions[i]}"
  UBUNTU_VERSION="${ubuntu_versions[i]}"
  CUDA_VERSION="${cuda_versions[i]}"
  gcloud builds submit --config cloudbuild.yaml . --substitutions=_ROS_DISTRO="$ROS_DISTRO",_CUDA_VERSION="$CUDA_VERSION" --timeout=86400 &
  echo Dispached ROS "$ROS_DISTRO" on ubuntu "$UBUNTU_VERSION" using nvidia/cuda:"$CUDA_VERSION" base image
done