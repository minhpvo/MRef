// Copyright ETHZ 2017
#include <OpenMesh/Core/IO/MeshIO.hh>

#include <OpenMesh/Core/IO/writer/PLYWriter.hh>
#include <OpenMesh/Core/IO/writer/OBJWriter.hh>
#include <OpenMesh/Core/IO/reader/PLYReader.hh>
#include <OpenMesh/Core/IO/reader/OBJReader.hh>
#include "MyMesh.h"

#include <string>
#include <boost/filesystem.hpp>
#include <iostream>

#include "MeshIO.h"

namespace MeshIO {

  void readMesh(MyMesh &ommesh, std::string savename, bool hasvertcolor, bool hasfacecolor, bool hasvertnormal,
                bool hasfacetexture, bool hasclasses) {

    // Check which extension
    std::string extension = boost::filesystem::extension(savename);
    if (extension == ".obj") {
      OpenMesh::IO::_OBJWriter_();
    } else if (extension == ".ply") {
      OpenMesh::IO::_PLYWriter_();
    } else {
      std::cout << "\n MeshUtil::readReadMeshVFN: could not figure extension. " << extension;
      exit(1);
    }

    OpenMesh::IO::Options ropt, wopt;
    if (hasfacecolor) {
      ommesh.request_face_colors();
      ropt += OpenMesh::IO::Options::FaceColor;
    }
    if (hasvertcolor) {
      ommesh.request_vertex_colors();
      ropt += OpenMesh::IO::Options::VertexColor;
      ropt += OpenMesh::IO::Options::ColorAlpha;
      if (!ommesh.has_vertex_colors()) {
        std::cout << "ERROR: Standard vertex property 'Colors' not available!" << std::endl;
      }
    }
    if (hasvertnormal) {
      ommesh.request_vertex_normals();
      ropt += OpenMesh::IO::Options::VertexNormal;
      // If the file did not provide vertex normals, then calculate them
      if (!ropt.check(OpenMesh::IO::Options::VertexNormal)) {
        // we need face normals to update the vertex normals
        ommesh.request_face_normals();
        // let the mesh update the normals
        ommesh.update_normals();
      }

    }
    if (hasfacetexture) {
      ommesh.request_halfedge_texcoords2D();
      ommesh.request_face_texture_index();
      ropt += OpenMesh::IO::Options::FaceTexCoord;
    }
    if (hasclasses) {
      std::cout << "MeshIO.cpp: Make sure that the classifications (uint8_t) are saved with column name  'alpha' "
                << std::endl;
    }

    if (extension == ".obj") {

      OpenMesh::IO::_OBJReader_();
      if (!OpenMesh::IO::read_mesh(ommesh, savename.c_str(), ropt)) {
        std::cerr << "Error loading obj mesh from file " << savename << std::endl;
      }
    } else if (extension == ".ply") {
      OpenMesh::IO::_PLYReader_();
      //  For now, ply files must be ascii, or else we can't read in the
      //    classifications. Make sure that the classifications (type uint8_t)
      //    are saved with column name 'alpha' as we need to hijack the 4th color
      //    channel in order to save it to the mesh vertex
      if (!OpenMesh::IO::read_mesh(ommesh, savename.c_str(), ropt)) {
        std::cerr << "Error loading ply mesh from file " << savename << std::endl;
      }
    }
  }

  void writeMesh(const MyMesh &mesh, const std::string &savename,
                 const bool hasvertcolor, const bool hasfacecolor,
                 const bool hasvertnormal, const bool hasfacetexture,
                 const bool hasclasses) {

    // Check which extension
    std::string extension = boost::filesystem::extension(savename);
    if (extension == ".obj") {/* OpenMesh::IO::_OBJWriter_(); */}
    else if (extension == ".ply") { /*OpenMesh::IO::_PLYWriter_(); */}
    else {
      std::cout << "\n MeshIO::writeMesh: could not figure extension.";
      exit(1);
    }

    OpenMesh::IO::Options wopt;
    if (hasfacecolor) wopt += OpenMesh::IO::Options::FaceColor;
    if (hasvertcolor) wopt += OpenMesh::IO::Options::VertexColor;
    if (hasvertcolor) wopt += OpenMesh::IO::Options::ColorAlpha;
    if (hasvertnormal) wopt += OpenMesh::IO::Options::VertexNormal;
    if (hasfacetexture) wopt += OpenMesh::IO::Options::FaceTexCoord;
    std::cout << "MeshUtil: Writing mesh\n";
    if (!OpenMesh::IO::write_mesh(mesh, savename.c_str(), wopt)) {
      std::cout << "MeshUtil: Write error\n";
      exit(1);
    }
  }
}
