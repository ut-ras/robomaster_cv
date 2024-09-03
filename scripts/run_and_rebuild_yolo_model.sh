#!/bin/bash
if [ -e /robomaster_cv/models/12_1_23-87_8.plan ]
then
    echo "The file /robomaster_cv/models/12_1_23-87_8.plan still exists. Please move or remove this file before building a new engine file."
    exit
fi



ros2 launch isaac_ros_yolov8 isaac_ros_yolov8_visualize.launch.py model_file_path:=/robomaster_cv/models/12_1_23-87_8.onnx engine_file_path:=/robomaster_cv/models/12_1_23-87_8.plan input_binding_names:=['images'] output_binding_names:=['output0'] network_image_width:=640 network_image_height:=640 force_engine_update:=True image_mean:=[0.0,0.0,0.0] image_stddev:=[1.0,1.0,1.0] input_image_width:=640 input_image_height:=640 confidence_threshold:=0.25 nms_threshold:=0.45