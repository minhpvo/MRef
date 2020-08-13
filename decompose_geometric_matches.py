import json
# from itertools import chain
import numpy as np
import scipy.sparse as sparse

data_folder        = "data_sr_low/"
geo_matches_file   = data_folder+"geometric_matches"
geo_matches_text   = data_folder+"geometric_matches_decomp.txt"
sfm_data_file      = data_folder+"sfm_data.json"
imglist_file       = data_folder+"imglist.txt"

# Reading the files
with open(geo_matches_file,'r') as matches_data:
  data = matches_data.readlines()
with open(sfm_data_file,'r') as sfm_data:
  sfm = json.load(sfm_data)
with open(imglist_file,'r') as imglist:
  imgs = imglist.readlines()

for ii, img in enumerate(imgs):
  imgs[ii] = img.strip().split('/')[-1]


views = sfm['views']
sfm_map = {}
img_map = {}
for view in views:
  v_id     = int(view['key'])
  png_name = view['value']['ptr_wrapper']['data']['filename']
  if png_name in imgs: 
    png_id        = imgs.index(png_name)
    sfm_map[v_id] = png_id   #pngs are indexed by 1, whereas "keys" and adj matrix are indexed at 0
    img_map[v_id] = png_name #pngs are indexed by 1, whereas "keys" and adj matrix are indexed at 0
  # orilist.write("ori/{0}.or\n".format(png_id))
  # imglist.write("img/{0}\n".format(png_name))
# orilist.close()
# imglist.close()


# trim preamble and brackets
data = data[2:-1]
adj = np.zeros(shape=(len(data),3))
for iadj, dat in enumerate(data):
  i,j = dat.replace("n","").strip().split("--")
  # entry = [int(i), int(j), 1]
  # if int(i) in 
  if ((int(i) in sfm_map.keys()) and (int(j) in sfm_map.keys())):
  # if ((img_map[int(i)] in imgs) and (img_map[int(j)] in imgs)):
    entry = [sfm_map[int(i)], sfm_map[int(j)], 1]
    adj[iadj,:] = entry

ndim = int(adj.max(axis=0)[:2].max()+1)
coo = sparse.coo_matrix((adj[:, 2], (adj[:, 0], adj[:, 1])), shape=[ndim,ndim],
                        dtype=int)

xx=coo.todense()
zz=xx+xx.T
# zz=np.triu(zz,0) 

f = open(geo_matches_text, "w")
f.write("  adjacency<<   ")

for iw, wline in enumerate(zz):
  ll = str(wline).replace("\n","").split(']')[0].split('[')[-1].split()
  pl = ",".join(ll)
  f.write(pl)
  if iw==ndim-1:
    f.write(';\n    ')
  else:
    f.write(',\n      ')
f.close()


##  Example: geometric_matches_decomp.txt
#  adjacency<<   
#    1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
#    1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,
#    0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,
#    0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,
#    0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
#    0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,
#    0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,
#    0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,
#    0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,
#    0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,
#    0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,
#    0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,
#    0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
#    0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,
#    0,0,0,0,0,0,0,0,0,0,0,0,0,1,1;
