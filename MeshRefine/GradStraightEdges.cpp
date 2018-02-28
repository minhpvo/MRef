// Copyright ETHZ 2017
// author: maros, 2016, maros.blaha@geod.baug.ethz.ch
// author: mathias, 2017. mathias.rothermel@geod.baug.ethz.ch

#include "GradStraightEdges.h"

// Constructur directely using mesh, mesh has face labels already
GradStraightEdges::GradStraightEdges(MyMesh mesh)
    	:
	_mesh(mesh)
{
	_grad.resize(mesh.n_vertices(),3);

}

GradStraightEdges::~GradStraightEdges()
{
}

Eigen::MatrixXf GradStraightEdges::getGrad() const
{
    return _grad;
}
void GradStraightEdges::oneRingStraightEdge2()
{
	// Reset grad
    	_grad.setZero();

	int numtransitions;
       	float a, b, c, gamma, w;
	unsigned short l1,l2;
	Eigen::MatrixXf pts(2,3);
	MyMesh::Point pv,pe;

	// Skim verts
	int i=-1;
	for (MyMesh::VertexIter v_it=_mesh.vertices_begin(); v_it!=_mesh.vertices_end(); ++v_it)
	{
	    	i++;
		if (!_mesh.is_boundary(*v_it))
		{
	    		// Skim edges
	    		numtransitions=0;
			for (MyMesh::VertexIHalfedgeIter vhe_it=_mesh.vih_iter(*v_it); vhe_it.is_valid(); ++vhe_it)
			{
				// Get faces
			    	auto fh1 = _mesh.face_handle(*vhe_it);
			        auto fh2 = _mesh.opposite_face_handle(*vhe_it);

				l1=_mesh.data(fh1).labelid();
				l2=_mesh.data(fh2).labelid();

				if(l1!=l2)
				{
					numtransitions++;
					if (numtransitions>2){ break; }

					// Store edge point
					pe=_mesh.point(_mesh.from_vertex_handle(*vhe_it));
					pts(numtransitions-1,0)=pe[0];
					pts(numtransitions-1,1)=pe[1];
					pts(numtransitions-1,2)=pe[2];
				}
			}
			// Finished edges
			if(numtransitions !=2){ continue; }

    			// If num boundary edges is 2 then we have a case
			pv=_mesh.point(*v_it);

                	// Compute weight Maros stuff.. why do we do this? 
                	a = sqrt(pow((pts(0,0)-pv[0]),2.0f)+pow((pts(0,1)-pv[1]),2.0f)+pow((pts(0,2)-pv[2]),2.0f));
                	b = sqrt(pow((pts(1,0)-pv[0]),2.0f)+pow((pts(1,1)-pv[1]),2.0f)+pow((pts(1,2)-pv[2]),2.0f));
                	c = sqrt(pow((pts(0,0)-pts(1,0)),2.0f)+pow((pts(0,1)-pts(1,1)),2.0f)+pow((pts(0,2)-pts(1,2)),2.0f));
                	gamma = acos((pow(a,2.0f)+pow(b,2.0f)-pow(c,2.0f))/(2*a*b))/PI*180; // angle at class transition vertex
                	w = -(1.0f/180.0f)*gamma+1.0f;

			// Compute Grad
			_grad(i,0)=((pts(0,0)+pts(1,0))*0.5-pv[0])*w;
			_grad(i,1)=((pts(0,1)+pts(1,1))*0.5-pv[1])*w;
			_grad(i,2)=((pts(0,2)+pts(1,2))*0.5-pv[2])*w;
		}
	}
}

// CONSTRUCTOR
GradStraightEdges::GradStraightEdges(std::vector<std::vector<float>> vertices, std::vector<std::vector<int>> faces, std::vector<int> labels) {

    // assign vertices and faces, generate openMesh object, compute normals
    _vertices = vertices;
    _verticesmov = vertices;
    _faces = faces;
    _labels = labels;

    // matrix with vertex movement
    Eigen::MatrixXf vertmovsumpre(vertices.size(),3);
    _vertmovsum = vertmovsumpre;

    // generate mesh with openmesh
    int i=0;
    std::vector<short> labels16(_labels.size());
    for(std::vector<short>::iterator it=labels16.begin();it!=labels16.end();++it){	*it=_labels[i]; i++; }
    //MeshConvertion::vectorsToOpenMesh(_vertices,_faces,labels16,_mesh);

} // END CONSTRUCTOR

// ====================================================================================


// ====================================================================================

// GET FACE LABELS FROM VERTEX LABELS
void GradStraightEdges::faceLabels() {

   // LabelVertFaceConvertion converter(&_mesh);
   //  converter.process(LabelVertFaceConvertion::FREQUENCY);


    // correct face labels
    // iterate over all faces
    for (MyMesh::FaceIter f_it=_mesh.faces_begin(); f_it!=_mesh.faces_end(); ++f_it) {

        int counterbuild, counterveg, counterroof;
        counterbuild = 0;
        counterveg = 0;
        counterroof = 0;

        for (MyMesh::FaceVertexIter fv_it=_mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it) {

            // get vertex ID
            int indv=fv_it->idx();

            // check classes
            if (_labels[indv] == 2) {
                counterbuild += 1;
            } // end check classes
            else if (_labels[indv] == 3) {
                counterveg += 1;
            }
            else if (_labels[indv] == 4) {
                counterroof += 1;
            }

        } // end vertex face iterator

        // correct
        if (counterbuild > 0) {
            _mesh.data(*f_it).setlabelid(2);
        } // end correct
        if (counterveg > 0) {
            _mesh.data(*f_it).setlabelid(3);
        }
        if (counterroof > 0) {
            _mesh.data(*f_it).setlabelid(4);
        }
    } // end face iterator

    // orginally from function "copyFacelabelToFaceColor"
    _mesh.request_face_colors();
    // skim faces
    for( MyMesh::FaceIter f_it=_mesh.faces_begin(); f_it!=_mesh.faces_end(); ++f_it)
    {

        MyMesh::Color col;//( mesh.data(*f_it).labelid(), mesh.data(*f_it).labelid(), mesh.data(*f_it).labelid() );
        //if( _mesh.data(*f_it).labelid()==1){col[0]=127; col[1]=127; col[2]=127;}
        //if( _mesh.data(*f_it).labelid()==2){col[0]=255; col[1]=0; col[2]=0;}
        //if( _mesh.data(*f_it).labelid()==3){col[0]=0; col[1]=0; col[2]=0;}
        //if( _mesh.data(*f_it).labelid()==4){col[0]=0; col[1]=0; col[2]=255;}

        if( _mesh.data(*f_it).labelid()==1){col[0]=127; col[1]=127; col[2]=127;}
        if( _mesh.data(*f_it).labelid()==2){col[0]=150; col[1]=0; col[2]=150;}
        if( _mesh.data(*f_it).labelid()==3){col[0]=0; col[1]=200; col[2]=0;}
        if( _mesh.data(*f_it).labelid()==4){col[0]=200; col[1]=200; col[2]=0;}

        _mesh.set_color(*f_it, col);
    }
}




// END GET FACE LABELS FROM VERTEX LABELS
// ====================================================================================
// MAKE EDGE ALONG CLASS BOUNDARIES STRAIGHT

// straight edge for 1 ring
void GradStraightEdges::oneRingStraightEdge() {

    // start timer
    PRSTimer timer; timer.start();

    // vector contains vertices to be moved
    Eigen::VectorXi vertmov = Eigen::VectorXi::Zero(_vertices.size());

    // iter over all vertices, determine which need to be moved Matze: so this counts changes of different labelsi
    // Case 0 1 2 2 2 2 would give a 2 but shouldnt be modified?
    for (MyMesh::VertexIter v_it=_mesh.vertices_begin(); v_it!=_mesh.vertices_end(); ++v_it){

        // get vertex ID
        //MyMesh::VertexHandle vhandle = v_it.handle();
        int indv = v_it->idx();
        std::string indvstr = std::to_string(indv);

        int facenr; facenr = 0; // face ID within the ring
        int lab, labpre; // initialize face labels

        // iter over 1-ring faces, determine which need to be moved
        for (MyMesh::VertexFaceIter vf_it=_mesh.vf_iter(*v_it); vf_it.is_valid(); ++vf_it) {

            // get face ID
            //MyMesh::FaceHandle fhandle = vf_it.handle();
            int indf = vf_it->idx();

            lab = _mesh.data(*vf_it).labelid();// get label ID

            facenr += 1; // increase face ID

            // from the second face on check class transitions Matze: isnt this statement always true?
            if (facenr > 1) {

                // class transition
                if (lab != labpre) {
                    vertmov(indv) += 1;
                } // end class transition

            } // end from the second face on check class transitions

            labpre = lab;

        } // end iter over 1-ring faces, determine which need to be moved
    } // end iter over all vertices, determine which need to be moved

    // iter again over all vertices, move vertices
    for (MyMesh::VertexIter v_it=_mesh.vertices_begin(); v_it!=_mesh.vertices_end(); ++v_it){

        // get vertex ID
        //MyMesh::VertexHandle vhandle = v_it.handle();
        int indv = v_it->idx();
        std::string indvstr = std::to_string(indv);

        // set vector with vertex movements to zero
        _vertmovsum(indv,0) = 0.0f; _vertmovsum(indv,1) = 0.0f; _vertmovsum(indv,2) = 0.0f;

        // consider only edges that need to be moved
        if (vertmov(indv) == 1 || vertmov(indv) == 2) {

            std::vector<int> transvertid;
            int transvertct = 0;

            // iter over all edges
            for (MyMesh::VertexEdgeIter ve_it=_mesh.ve_iter(*v_it); ve_it.is_valid(); ++ve_it) {

                // get edge ID
                //MyMesh::EdgeHandle ehandle = ve_it.handle();
                int inde = ve_it->idx();

                // split edge into half edges
                MyMesh::HalfedgeHandle h0 = _mesh.halfedge_handle(*ve_it,0);
                MyMesh::HalfedgeHandle h1 = _mesh.halfedge_handle(*ve_it,1);

                // get faces of halfedges
                MyMesh::FaceHandle f0 = _mesh.opposite_face_handle(h0);
                MyMesh::FaceHandle f1 = _mesh.opposite_face_handle(h1);

                // convert face IDs to integers
                int indf0 = f0.idx();
                int indf1 = f1.idx();

                // continue only for edges which are not at the border
                if (indf0 >= 0 && indf1 >= 0) {

                    // get face labels
                    int lab0, lab1;
                    lab0 = _mesh.data(f0).labelid();
                    lab1 = _mesh.data(f1).labelid();

                    //std::cout << "lab 0: " << lab0 << ", lab 1: " << lab1 << std::endl;

                    // check if edge adjoins two classes
                    if (lab0 != lab1) {

                        // get vertices of halfedges
                        MyMesh::VertexHandle v0 = _mesh.to_vertex_handle(h0);
                        MyMesh::VertexHandle v1 = _mesh.to_vertex_handle(h1);

                        //std::cout << "v0: " << v0 << ", v1: " << v1 << std::endl;

                        // convert to ints
                        int indv0 = v0.idx();
                        int indv1 = v1.idx();

                        // save non central vertex to transition vertex
                        if (indv0 != indv) {
                            transvertid.push_back(indv0); // save vertex ID
                            transvertct += 1; // move to next transisiotn vertex
                        }
                        else if (indv1 != indv) {
                            transvertid.push_back(indv1); // save vertex ID
                            transvertct += 1; // move to next transisiotn vertex
                        }
                    } // end check if edge adjoins two classes
                } // end continue only for edges which are not at the border
            } // end iter over all edges

            // move only edges with 2 transitions
            if (transvertid.size() == 2) {

                // edge with classe transitions
                Eigen::Vector3f edge, edgev0, edgev1, vertcen, grad;

                // ingredients for grad vector
                vertcen(0) = _vertices[indv][0];
                vertcen(1) = _vertices[indv][1];
                vertcen(2) = _vertices[indv][2];

                edgev0(0) = _vertices[transvertid[0]][0];
                edgev0(1) = _vertices[transvertid[0]][1];
                edgev0(2) = _vertices[transvertid[0]][2];

                edgev1(0) = _vertices[transvertid[1]][0];
                edgev1(1) = _vertices[transvertid[1]][1];
                edgev1(2) = _vertices[transvertid[1]][2];

                // point where grad vec is pointing to
                edge = edgev0+(edgev1 - edgev0)*0.5f;

                // grad vector
                grad = edge - vertcen;

                // compute weight
                float a, b, c, gamma, w;
                a = sqrt(pow((edgev0(0)-vertcen(0)),2.0f)+pow((edgev0(1)-vertcen(1)),2.0f)+pow((edgev0(2)-vertcen(2)),2.0f));
                b = sqrt(pow((edgev1(0)-vertcen(0)),2.0f)+pow((edgev1(1)-vertcen(1)),2.0f)+pow((edgev1(2)-vertcen(2)),2.0f));
                c = sqrt(pow((edgev0(0)-edgev1(0)),2.0f)+pow((edgev0(1)-edgev1(1)),2.0f)+pow((edgev0(2)-edgev1(2)),2.0f));
                gamma = acos((pow(a,2.0f)+pow(b,2.0f)-pow(c,2.0f))/(2*a*b))/PI*180; // angle at class transition vertex
                w = -(1.0f/180.0f)*gamma+1.0f;
                //w = std::max(std::min(w,1.0f),0.000001f);

                _vertmovsum(indv,0) = grad(0)*w;
                _vertmovsum(indv,1) = grad(1)*w;
                _vertmovsum(indv,2) = grad(2)*w;

            } // end move only edges with 2 transitions
        } // end consider only edges that need to be moved
    } // end iter again over all vertices, move vertices

    timer.stop();
    //std::cout<< "Edge straightening took: "<< timer.getTimeSec()<<" sec" << std::endl;

}

// END MAKE EDGE ALONG CLASS BOUNDARIES STRAIGHT
// ====================================================================================

// GET OPTIMIZED RESULTS
Eigen::MatrixXf GradStraightEdges::straightEdgeMov() {
    return _vertmovsum;
}

// END GET OPTIMIZED RESULTS
// ====================================================================================













