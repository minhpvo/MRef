// Copyright ETHZ 2017
// author: mathias rothermel, 2016, mathias.rothermel@geod.baug.ethz.ch

#include "MeshConv.h"
#include <iostream>

namespace MeshConv
{

void openMeshToVectors( const MyMesh &ommesh, std::vector<std::vector<float> > &verts, std::vector<std::vector<int> > &faces, std::vector<short> &labels)
{

	int numverts=ommesh.n_vertices();
	int numfaces=ommesh.n_faces();
	int v,i;

	// Resize vectors
	verts.resize(numverts); for(i=0; i<numverts; i++){ verts[i].resize(3); }
	faces.resize(numfaces); for(i=0; i<numfaces; i++){ faces[i].resize(3); }
	labels.resize(numfaces);

	// Copy verts and labels
	int vertcount=0;
	for (MyMesh::VertexIter v_it=ommesh.vertices_begin(); v_it!=ommesh.vertices_end(); ++v_it){
		verts[vertcount][0]=ommesh.point(*v_it)[0];
		verts[vertcount][1]=ommesh.point(*v_it)[1];
		verts[vertcount][2]=ommesh.point(*v_it)[2];
		vertcount++;
	}

	// Copy faces
	int facecount=0;
	MyMesh::CFVIter fv_it;
	for (MyMesh::FaceIter f_it=ommesh.faces_begin(); f_it!=ommesh.faces_end(); ++f_it)
	{
   		v=0;
    		for(fv_it=ommesh.cfv_iter(*f_it); fv_it.is_valid(); ++fv_it)
		{
    			faces[facecount][v]=fv_it->idx(); 
			v++;
	   	}
		labels[facecount]=ommesh.data(*f_it).labelid();
    facecount++;
	}
}

void vectorsToOpenMesh(const std::vector<std::vector<float> > &verts, const std::vector<std::vector<int> > &faces, const std::vector<short> &labels, MyMesh &ommesh )
{

	int numfaces=faces.size();
	int numverts=verts.size();

	// Copy Verts
	MyMesh::VertexHandle* vhandles=new MyMesh::VertexHandle[numverts];
	int v=0;
	for(std::vector<std::vector<float> >::const_iterator it=verts.begin(); it !=verts.end(); ++it){
	    vhandles[v] = ommesh.add_vertex(MyMesh::Point((*it)[0],(*it)[1],(*it)[2])); v++;
	}
	// Copy Faces
	std::vector<MyMesh::VertexHandle> face_vhandles(3); v=0;
	for( std::vector<std::vector<int> >::const_iterator it=faces.begin(); it!=faces.end(); ++it){

		face_vhandles[0]=vhandles[(*it)[0]];
		face_vhandles[1]=vhandles[(*it)[1]];
		face_vhandles[2]=vhandles[(*it)[2]];
		ommesh.add_face(face_vhandles); v++; 
	}
	delete [] vhandles;

	// Copy Labels
	int f=0;
	for (MyMesh::FaceIter f_it=ommesh.faces_begin(); f_it!=ommesh.faces_end(); ++f_it)
	{
	    ommesh.data(*f_it).setlabelid(labels[f]); f++;
	}
}

void faceColorToFaceLabel(MyMesh &mesh)
{
    	unsigned char first, second;
	unsigned short s;

    	for (MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it)
	{
		const MyMesh::Color& c=mesh.color(*f_it);
		first=c[0];
		second=c[1];

//		std::cout << "\n first="<<(int)first<<" second="<<(int)second;
		s=(((unsigned short)first) << 8) | second;
		mesh.data(*f_it).setlabelid(s);

	}
}

void vertexAlphaToVertexLabel(MyMesh &mesh)
{
  unsigned char r, g, b, al;
  int s;
	// unsigned short s;

  for(MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
	{
		const MyMesh::Color& c=mesh.color(*v_it);
		r=c[0];
		g=c[1];
		b=c[2];
		al=c[3];

		s = static_cast<int>(al);
		mesh.data(*v_it).setclassification(s);
    mesh.set_color(*v_it, MyMesh::Color(r,g,b,al));
    
	}
}

void vertexRGBToVertexGreyLabel(MyMesh &mesh)
{
  unsigned char r, g, b, al;
  int s;
  float rgbavg;
	// unsigned short s;

  for(MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
	{
		const MyMesh::Color& c=mesh.color(*v_it);
		r=c[0];
		g=c[1];
		b=c[2];
		al=c[3];
		rgbavg = static_cast<float>(r+g+b)/3.0;

		s = static_cast<int>( rgbavg*6.0/255.0 );
		mesh.data(*v_it).setclassification(s);
    mesh.set_color(*v_it, MyMesh::Color(r,g,b,al));
    
	}
}

void vertexLabelToFaceLabel(MyMesh &mesh)
{
	// Assign first vertex class label to the face label. 
	// This could of course be done better, but it's
	//  not as important currently,since we are already relabeling
	//  and the initial class labels are more an initial guess.
	//  Note that this is at least consistent with writing mesh class output
	int vertexclass = 0;
	// Option to save vertex colors at the same time...mostly untested code though
  //	unsigned char r, g, b, al;
	MyMesh::FaceVertexIter fv_it;
	for (MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it) {
		for(fv_it=mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it) {
  		vertexclass = mesh.data(*fv_it).classification();
			// vertexcolor = mesh.color(*fv_it);
      // const MyMesh::Color& c=mesh.color(*fv_it);
      // r=c[0]; g=c[1]; b=c[2]; al=c[3];
  		break;
  	}
  	mesh.data(*f_it).setlabelid(static_cast<short>(vertexclass));
    // mesh.set_color(*f_it, MyMesh::Color(r,g,b,al));
  }
}


void vertexColorToFaceColorICCV(MyMesh &mesh)
	{
		// Assign first vertex RGB color to the face color.
		// TODO (MAC): This could of course be done better
		MyMesh::FaceVertexIter fv_it;
		for (MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it) {
			for(fv_it=mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it) {
				mesh.set_color(*f_it, mesh.color(*fv_it));
				break;
			}
		}
	}

void vertexLabelToVertexColorICCV(MyMesh &mesh)
{
	int s;
	for(MyMesh::VertexIter v_it=mesh.vertices_begin(); v_it!=mesh.vertices_end(); ++v_it)
	{
		s = mesh.data(*v_it).classification();
		if(s==0) // Unknown
		{
			mesh.set_color(*v_it, MyMesh::Color(0 ,0, 0, s));
		}
		else if(s==1) //Mobile objects like planes, trains, cars, animals, and people
		{
			mesh.set_color(*v_it, MyMesh::Color(255, 0, 0, s));
		}
		else if(s==2) // High vegetation like trees (~>20cm)
		{
			mesh.set_color(*v_it, MyMesh::Color(0,255,0, s));
		}
		else if(s==3) // Natural ground (~<20cm)
		{
			mesh.set_color(*v_it, MyMesh::Color(205,133,63, s));
		}
		else if(s==4) // Ground-level man-made objects like pavement
		{
			mesh.set_color(*v_it, MyMesh::Color(255, 255, 0, s));
		}
		else if(s==5) // Buildings and man-made structures rising above ground-level
		{
			mesh.set_color(*v_it, MyMesh::Color(255, 255, 255, s));
		}
		else if(s==6) // Water
		{
			mesh.set_color(*v_it, MyMesh::Color(0, 0, 255, s));
		}
	}
}

void faceLabelToFaceColorICCV(MyMesh &mesh)
{
	unsigned short s;
	for(MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it)
	{
		s=mesh.data(*f_it).labelid();
		if(s==0) // Unknown
		{
			mesh.set_color(*f_it, MyMesh::Color(0 ,0, 0,s));
		}
		else if(s==1) //Mobile objects like planes, trains, cars, animals, and people
		{
			mesh.set_color(*f_it, MyMesh::Color(255, 0, 0,s));
		}
		else if(s==2) // High vegetation like trees (~>20cm)
		{
			mesh.set_color(*f_it, MyMesh::Color(0,255,0,s));
		}
		else if(s==3) // Natural ground (~<20cm)
		{
			mesh.set_color(*f_it, MyMesh::Color(205,133,63,s));
		}
		else if(s==4) // Ground-level man-made objects like pavement
		{
			mesh.set_color(*f_it, MyMesh::Color(255, 255, 0,s));
		}
		else if(s==5) // Buildings and man-made structures rising above ground-level
		{
			mesh.set_color(*f_it, MyMesh::Color(255, 255, 255,s));
		}
		else if(s==6) // Water
		{
			mesh.set_color(*f_it, MyMesh::Color(0, 0, 255,s));
		}
	}
}

void faceColorICCVToFacelabel(MyMesh &mesh)
{
	unsigned short s;
	for(MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it)
	{
		if(mesh.color(*f_it)[0]==150) //Facade
		{
			mesh.data(*f_it).setlabelid(0);
		}
		else if(mesh.color(*f_it)[0]==60) //Ground
		{
			mesh.data(*f_it).setlabelid(1);
		}
		else if(mesh.color(*f_it)[0]==0 && mesh.color(*f_it)[2]==0) // Vegetation
		{
			mesh.data(*f_it).setlabelid(2);
		}
		else if(mesh.color(*f_it)[0]==200) // Roof
		{
			mesh.data(*f_it).setlabelid(3);
		}
		else if(mesh.color(*f_it)[0]==0 && mesh.color(*f_it)[2]==200) // Vegetation
		{
			mesh.data(*f_it).setlabelid(4);
		}
	}
}

void faceLabelToFaceColorRandom(MyMesh &mesh)
{
	unsigned short s;
	std::vector<MyMesh::Color> colorvec(mesh.n_faces());

	//Build vector with random numbers 
	for(int i=0;i<mesh.n_faces();i++)
	{
	    colorvec[i][0]=rand()%255;
	    colorvec[i][1]=rand()%255;
	    colorvec[i][2]=rand()%255;
		  colorvec[i][2]=rand()%255;
	}
	for(MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it)
	{
		s=mesh.data(*f_it).labelid();
		mesh.set_color(*f_it, colorvec[s]);
	}
}

} // namespace 

