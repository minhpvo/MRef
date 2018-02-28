# MeshRef #

Meshref is an implementation of the algorithm described in our paper "Semantically Informed Multiview Surface Refinement".

If you use the code in your work please cite:
```
@InProceedings{Blaha_2017_ICCV,
     author  = {Blaha, Maros and Rothermel, Mathias and Oswald, Martin Ralf and Sattler, Torsten and Richard, Audrey and Wegner, Jan D. and Pollefeys, Mark and Schindler, Konrad},
     title   = {Semantically Informed Multiview Surface Refinement},
     journal = {The IEEE International Conference on Computer Vision (ICCV)},
     month   = {October},
     year    = {2017}
}
```
If you use the data in your work please cite:
```
@misc{slagboom,
     author = {{Slagboom en Peeters Aerial Survey}},
     note ={\url{http://www.slagboomenpeeters.com/3d.htm.}}
}
```
The labeling section contains parts of Nils Moehlres and Michael Waechters [code](https://github.com/nmoehrle/mvs-texturing) described in this [paper](https://link.springer.com/chapter/10.1007/978-3-319-10602-1_54)
### Dependencies ###
1. [Eigen](http://eigen.tuxfamily.org/)
2. [OpenMesh](http://www.openmesh.org/)
3. [OpenCV](https://opencv.org/)
4. [Boost](http://www.boost.org/)
5. [Embree/TBB](https://embree.github.io/)

### Compilation ###
* Install dependencies
* Clone the repository

`cd yourpath`

`git clone https://mathiaro@bitbucket.org/mathiaro/meshref.git`

`cd meshref`

* Copy Makefile_copy.rules to Makefile.rules 

`cp Makefile_copy.rules Makefile.rules`

* Adapt the include/link flags for Eigen, Boost, OpneCV, OpenMesh, Embree (marked by ###)

* Run

`make lib`

`make exe`

* The executable should be located in `yourpath/meshref/bin`
 
### Running MeshRef ###
* Download the [data](https://polybox.ethz.ch/index.php/s/CXWNqjwQJYcHZ4O)
* Make a data directory `yourpath/meshref/data`

`mkdir yourpath/meshref/data`

* unpack the contents of the data package into the data dirctory

`tar -xzf data.tar.gz -C yourpath/meshref/data`

* Execute meshref in the bin folder and specify your processing path

`cd /yourpath/meshref/bin`

`./meshref -b ../data`

* All important settings can be adapted in the parameter file `yourpath/meshref/data/ControlRefine.txt`, see the file param_descr.txt for an explanation of the individual parameters.  
### License ###
Our software is licensed under the BSD 3-Clause license, for more details see the LICENSE.txt file.

### Bugs and Questions ###

* Feel free to contact us for [questions or bug reports](mailto:mathias.rothermel@geod.baug.ethz.ch)!

