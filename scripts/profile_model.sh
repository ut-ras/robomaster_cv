#!/bin/bash
trtexec --onnx=/home/stampede/Documents/robomaster_cv/models/12_1_23-87_8.onnx \
        --fp16 \
        --saveEngine=/home/stampede/Documents/robomaster_cv/models/profiling/12_1_23-87_8.engine