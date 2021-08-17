#!/bin/bash

gst-launch-1.0  -v udpsrc port=5600 \
! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264 \
! rtph264depay \
! avdec_h264 \
! videoconvert \
! autovideosink fps-update-interval=1000 sync=false
