#!/bin/bash

# todo (MAC):
# Note this is NOT tested, and the hive_scripts only work
#  on a dev instance, with my testing branch. I am working
#  on cleaning it up

# model ID name that lives in the 'hostname' file system
#  this is the model mesh we'll use in meshref
model_id='5efe41c0505bb7b34c3fc3a1'
hostnme='mac-dev-127'
# where I want to keep all my output, that then serves as meshref data input
datafolder='/home/centos/MRef/data_sr_low'

# make meshref directories
mkdir $datafolder/temp
mkdir $datafolder/mesh
mkdir $datafolder/img
mkdir $datafolder/likeli
mkdir $datafolder/ori

# Script to get the transformed camera data and normalized mesh in floaating pt ascii
bazel run //hive/hive_scripts/lib/mesh:meshref_datagen -- $datafolder/temp $model_id

# grab the matches folder to extract geometirc matches file
sudo cp /mnt/HM/$hostname/hive/waggle/$model_id/reconstruction/matches.tar.gz $datafolder/temp
tar -xf $datafolder/temp/matches.tar.gz -C $datafolder/temp
cp $datafolder/temp/geometric_matches $datafolder
cp $datafolder/temp/temp/sfm_data.json $datafolder

#  grab the frames folder to extract the png frames
sudo cp /mnt/HM/$hostname/hive/waggle/$model_id/data/frames.tar.gz $datafolder/temp
tar -xf $datafolder/temp/frames.tar.gz -C $datafolder/img

# create empty files for writing source/dest image paths for inference engine
> $datafolder/segmentation_image_src.txt
> $datafolder/segmentation_image_dst.txt

##  keeping this in case I need to circle back to use it
# cd $datafolder/img
# for i in *.png 
# do
#   echo "$datafolder/img/$i" >> $datafolder/segmentation_image_src.txt
#   echo "$datafolder/likeli/$i" >> $datafolder/segmentation_image_dst.txt
# done

# take the image file listing and write the paths to create an image
# source and destination path text files for passing to the inference binary
find $datafolder/img -name "*.png" > $datafolder/segmentation_image_src.txt
cp $datafolder/segmentation_image_src.txt $datafolder/segmentation_image_dst.txt
# find/replace text, becasue input/output listings both have different file endings
sed -i 's/.png/.jpg/g' $datafolder/segmentation_image_src.txt
sed -i 's/.png/.tif/g' $datafolder/segmentation_image_dst.txt
# also the input and output folder is going to be the likeli folder, not 'img'
sed -i 's+/img/+/likeli/+g' $datafolder/segmentation_image_src.txt
sed -i 's+/img/+/likeli/+g' $datafolder/segmentation_image_dst.txt

# copy images, convert to jpg (for inference input), and call inference binary
cp $datafolder/img/* $datafolder/likeli
cd $datafolder/likeli
gm mogrify -format jpg *.png
rm *.png
inference --image=$datafolder/segmentation_image_src.txt --results=$datafolder/segmentation_image_dst.txt --scale=100 --graph=segmentation_model.pb --root_dir=/usr/etc/hive/segmentation/;

# make meshlist.txt file with the path to the input ply file
cd $datafolder
find mesh -name "*.ply" > $datafolder/meshlist.txt

# TODO: 
# create orientation files, and the image adjacency matrix. The adjacency
#  matrix script relies on the imglist.txt created in hive_to_ori, so it
#  needs to run first
# Note that for these 2 python scripts, you need to manually change the folder
#  string at the top so that you're looking in the right place. 
#  It's terrible; I'm sorry, I'll fix it eventually
# call: python hive_to_ori.py
#  you should now have: an imglist.txt, orilist.txt, likelilist.txt
#   and a populates ori/ folder with the camera matrices for each image
# call: python decompose_geometric_matches.py
#  you should now have: a geometric_matches_decomp.txt file with the
#  adjacency matrix needed in Consistency.cpp. You (currently) need to
#  copy/paste that text in, until we write in a file reader.

# TODO: delete the temp dir, but I haven't actually tested and don't want to code it yet