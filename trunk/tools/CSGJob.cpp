/*
  This file is part of SolidarityCSG.

  SolidarityCSG is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  SolidarityCSG is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with SolidarityCSG.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdexcept>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include "CSGJob.h"


using namespace csg;
using namespace std;
using namespace os;


//------------------------------------------------------------------------------
// CSGJob
//------------------------------------------------------------------------------

CSGJob::CSGJob()
{
  mCSGRoot = 0;
  mResolution = Vector3(0.1, 0.1, 0.1);
  mOutputFileName = string("output");
  mOutputType = otSlices;
  mOutputFormat = ofTGA;
}

CSGJob::~CSGJob()
{
  if(mCSGRoot)
    delete(mCSGRoot);
}

void CSGJob::LoadFromXML(const char * aFileName)
{
  // Load input XML
  cout << "Loading job description " << aFileName << "..." << endl;
  TiXmlDocument xmlDoc;
  if(!xmlDoc.LoadFile(aFileName))
    throw runtime_error(xmlDoc.ErrorDesc());

  // Parse XML document
  TiXmlElement * root = xmlDoc.RootElement();
  if(!root || (root->Type() != TiXmlNode::ELEMENT) ||
     (root->Value() != string("csg")))
    throw runtime_error("Could not find root element (\"csg\").");
  TiXmlNode * node = root->FirstChild();
  while(node)
  {
    if(node->Type() == TiXmlNode::ELEMENT)
    {
      // Settings node?
      if(node->Value() == string("settings"))
        LoadSettings(node);

      // Volume description node?
      else if(node->Value() == string("volume"))
        LoadVolume(node);

      // Unrecognized node?
      else
        cout << " Warning: Unrecognized node: " << node->Value() << endl;
    }

    // Next node...
    node = node->NextSibling();
  }
}

void CSGJob::Execute()
{
  double dt;

  // Set up a sample space
  mTimer.Push();
  cout << "Setting up voxel sample space..." << flush;
  SampleSpace space;
  BoundingBox sceneAABB;
  mCSGRoot->GetBoundingBox(sceneAABB);
  space.DefineSpace(sceneAABB, mResolution);
  mCSGRoot->SetSampleSpace(&space);
  dt = mTimer.PopDelta();
  cout << "done! (" << int(dt * 1000.0 + 0.5) << " ms)" << endl;

  // Prepare image writer
  ImageWriter * output = 0;
  if(mOutputFormat == ofTGA)
    output = new TGAImageWriter;
  else
    throw runtime_error("Unsupported output format.");
  try
  {
    output->SetFormat(space.mDiv[0], space.mDiv[1], ImageWriter::pfSigned8);

    // Perform operation...
    cout << "Executing job..." << flush;
    mTimer.Push();
    vector<Voxel> voxelSlice;
    voxelSlice.resize(space.mDiv[0] * space.mDiv[1]);
    for(int i = 0; i < space.mDiv[2]; ++ i)
    {
      // Generate slice data
      mCSGRoot->ComposeSlice(&voxelSlice[0], i);

      // Construct file name for the slice
      stringstream name;
      name << mOutputFileName;
      name.fill('0');
      name.width(5);
      name << i;
      name.width(0);
      if(mOutputFormat == ofTGA)
        name << ".tga";

      // Write this file to disk
      output->SetData(&voxelSlice[0]);
      output->SetSliceNo(i);
      output->SetSampleSpace(&space);
      output->SaveToFile(name.str().c_str());
    }
    dt = mTimer.PopDelta();
    cout << "done! (" << int(dt * 1000.0 + 0.5) << " ms)" << endl;

    delete output;
    output = 0;
  }
  catch(...)
  {
    if(output)
      delete output;
    throw;
  }
}

void CSGJob::LoadSettings(TiXmlNode * aNode)
{
  cout << " Reading settings..." << endl;
  TiXmlNode * node = aNode->FirstChild();
  while(node)
  {
    if(node->Type() == TiXmlNode::ELEMENT)
    {
      TiXmlElement * e = (TiXmlElement *) node;

      // Resolution node?
      if(node->Value() == string("resolution"))
      {
        e->QueryDoubleAttribute("x", &mResolution.x);
        e->QueryDoubleAttribute("y", &mResolution.y);
        e->QueryDoubleAttribute("z", &mResolution.z);
      }

      // Output description node?
      else if(node->Value() == string("output"))
      {
        const char * s;

        // Get output file name
        s = e->Attribute("name");
        if(s)
          mOutputFileName = string(s);

        // Get output type
        s = e->Attribute("type");
        if(s)
        {
          if(s == string("slices"))
            mOutputType = otSlices;
          else
            cout << "  Warning: Unrecognized output type: " << s << endl;
        }

        // Get output format
        s = e->Attribute("format");
        if(s)
        {
          if(s == string("tga"))
            mOutputFormat = ofTGA;
          else
            cout << "  Warning: Unrecognized output format: " << s << endl;
        }
      }

      // Unrecognized node?
      else
        cout << "  Warning: Unrecognized node: " << node->Value() << endl;
    }

    // Next node...
    node = node->NextSibling();
  }

  // Sanity check of the settings (e.g. format & type compatibility)
  // ...
}

void CSGJob::LoadVolume(TiXmlNode * aNode)
{
  cout << " Reading volume description..." << endl;
  TiXmlNode * node = aNode->FirstChild();
  while(node)
  {
    if(node->Type() == TiXmlNode::ELEMENT)
    {
      mCSGRoot = LoadCSGNode((TiXmlElement *) node);
      break;
    }
    node = node->NextSibling();
  }

  // Did we get a proper volume description?
  if(!mCSGRoot)
    throw runtime_error("No volume defined.");
}

CSGNode * CSGJob::LoadCSGNode(TiXmlElement * aElement)
{
  string nodeName = string(aElement->Value());
  if(nodeName == string("union"))
  {
    CSGUnion * result = new CSGUnion();
    TiXmlNode * node = aElement->FirstChild();
    while(node)
    {
      if(node->Type() == TiXmlNode::ELEMENT)
        ((CSGCompositeNode *)result)->AddChild(LoadCSGNode((TiXmlElement *) node));
      node = node->NextSibling();
    }
    return result;
  }
  else if(nodeName == string("intersection"))
  {
    CSGIntersection * result = new CSGIntersection();
    TiXmlNode * node = aElement->FirstChild();
    while(node)
    {
      if(node->Type() == TiXmlNode::ELEMENT)
        ((CSGCompositeNode *)result)->AddChild(LoadCSGNode((TiXmlElement *) node));
      node = node->NextSibling();
    }
    return result;
  }
  else if(nodeName == string("difference"))
  {
    CSGDifference * result = new CSGDifference();
    TiXmlNode * node = aElement->FirstChild();
    while(node)
    {
      if(node->Type() == TiXmlNode::ELEMENT)
        ((CSGCompositeNode *)result)->AddChild(LoadCSGNode((TiXmlElement *) node));
      node = node->NextSibling();
    }
    return result;
  }
  else if(nodeName == string("sphere"))
  {
    // Get shape parameters
    Vector3 c(0.0, 0.0, 0.0);
    double r = 1.0;
    aElement->QueryDoubleAttribute("cx", &c.x);
    aElement->QueryDoubleAttribute("cy", &c.y);
    aElement->QueryDoubleAttribute("cz", &c.z);
    aElement->QueryDoubleAttribute("r", &r);

    // Create shape node
    SphereVoxelize * v = new SphereVoxelize;
    v->SetSphere(c, r);
    CSGShape * result = new CSGShape();
    ((CSGShape *)result)->DefineShape(v);
    return result;
  }
  else if(nodeName == string("box"))
  {
    // Get shape parameters
    Vector3 c(0.0, 0.0, 0.0);
    Vector3 s(1.0, 1.0, 1.0);
    aElement->QueryDoubleAttribute("cx", &c.x);
    aElement->QueryDoubleAttribute("cy", &c.y);
    aElement->QueryDoubleAttribute("cz", &c.z);
    aElement->QueryDoubleAttribute("sx", &s.x);
    aElement->QueryDoubleAttribute("sy", &s.y);
    aElement->QueryDoubleAttribute("sz", &s.z);

    // Create shape node
    BoxVoxelize * v = new BoxVoxelize;
    v->SetBox(c, s);
    CSGShape * result = new CSGShape();
    ((CSGShape *)result)->DefineShape(v);
    return result;
  }
  else if(nodeName == string("mesh"))
  {
    // Load mesh from file
    const char * src = aElement->Attribute("src");
    if(!src)
      throw runtime_error("Missing src attribute in mesh node.");
    cout << "  Loading mesh file " << src << "..." << flush;
    mTimer.Push();
    STLMeshReader meshReader;
    Mesh mesh;
    meshReader.SetMesh(&mesh);
    meshReader.LoadFromFile(src);

    // Create shape node
    cout << "building..." << flush;
    MeshVoxelize * v = new MeshVoxelize;
    v->SetTriangles(mesh);
    CSGShape * result = new CSGShape();
    ((CSGShape *)result)->DefineShape(v);
    double dt = mTimer.PopDelta();
    cout << "done! (" << int(dt * 1000.0 + 0.5) << " ms)" << endl;
    return result;
  }
  else
  {
    string err = string("Invalid CSG node: ") + nodeName;
    throw runtime_error(err.c_str());
  }
}
