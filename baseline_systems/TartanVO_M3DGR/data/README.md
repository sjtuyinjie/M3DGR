rosrun image_view extract_images _sec_per_frame:=0.0096 image:=/camera/color/image_raw

rosrun image_transport republish compressed in:=/camera/color/image_raw raw out:=/camera/color/image_raw

