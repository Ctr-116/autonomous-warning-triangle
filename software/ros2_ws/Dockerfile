FROM ros:humble-ros-base-jammy
WORKDIR /workspace
COPY . /workspace
RUN bash scripts/install_dependencies.sh && bash scripts/build.sh
ENV ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=98
ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["source scripts/environment.sh && ros2 launch yahboomcar_bringup bringup.launch.py use_mock:=true rviz:=false"]
