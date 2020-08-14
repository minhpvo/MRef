import json
from itertools import chain


# TODO (MAC) This definitely needs to be read in as a CLI
#  esp. since the paths haven't been checked for consistency
data_folder   = 'data_sr_high/' 
camera_file   = data_folder+"camera_data_tf.json"
ori_folder    = "ori/"
img_folder    = "img/"
likeli_folder = "likeli/"
output_folder = data_folder+ori_folder

# Reading the json as a dict
with open(camera_file) as json_data:
  data = json.load(json_data)

intrinsics = data['intrinsics'][0]['value']['ptr_wrapper']['data']
fl = intrinsics['focal_length']
width = intrinsics['width']
height = intrinsics['height']
pp = intrinsics['principal_point']
sx, sy, sa = intrinsics['disto_k3']

ox, oy = pp[0], pp[1]
fsx = fl
fsy = fl
rc = [height, width]
s = 0

extrinsics = data['extrinsics']   
orilist = open(data_folder+"orilist.txt", "w")
imglist = open(data_folder+"imglist.txt", "w")
likelilist = open(data_folder+"likelilist.txt", "w")

views = data['views']

for entry in extrinsics:
  view_i = [ view['key']==entry['key'] for view in views].index(True)
  i = int(views[view_i]['value']['ptr_wrapper']['data']['filename'].split('.')[0])
  c =  entry['value']['center']
  r =  entry['value']['rotation']
  r =  list(chain.from_iterable(r))
  k = [fsx,   s, ox, 
         0, fsy, oy, 
         0,   0,  1]
  outfile = output_folder+"{0}.or".format(i)
  l1 = "$id {0:3}".format(i)
  l2 = "\n$rc{0:19}{1:19}".format(rc[0],rc[1])
  l3 = "\n$R"
  for ir in r:
    l3 = l3 + " {:18.8f}".format(ir)
  l4 = "\n$K"
  for ik in k:
    l4 = l4 + " {:18.8f}".format(ik)
  l5 = "\n$C"
  for ic in c:
    l5 = l5 + " {:18.10f}".format(ic)
  f = open(outfile, "w")
  for wline in [l1,l2,l3,l4,l5]:
    f.write(wline)
  f.close()
  orilist.write(ori_folder + "{0}.or\n".format(i))
  imglist.write(img_folder + "{0}.png\n".format(i))
  likelilist.write(likeli_folder + "{0}.tif\n".format(i))
orilist.close()
imglist.close()
likelilist.close()
##  Example: firstori.or
# $id none
# $rc               -1               -1
# $R       0.00000000       0.00000000       0.00000000       0.00000000       0.00000000       0.00000000       0.00000000       0.00000000       0.00000000
# $K       0.00000000       0.00000000       0.00000000       0.00000000       0.00000000       0.00000000       0.00000000       0.00000000       0.00000000
# $C         1.000000         2.000000         3.000000