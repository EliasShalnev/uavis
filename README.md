# uavis
This package defines UAV technical vision system simulation. Each scout model with vision system suppose to launch this node. Node should be executed with namespace /scout<number>.
##### Node arguments
\- -port - camera gstreamer port
\- -frameProcessingTime - frame processing time (in seconds) 
\- -secondKindError - false negative error for each target coordinate (from 0 to 1)
##### Launch
uavis.launch defines all default arguments and node execution. Include this launch file in UAV model spawning launch file to execute uavis node by default.