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

// Uncomment this line to disable multi threaded operation
//#define SINGLE_THREADED

#include <stdexcept>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include "CSGJob.h"
#ifndef SINGLE_THREADED
#include "OS/OSThread.h"
#endif


using namespace csg;
using namespace std;
using namespace os;


//------------------------------------------------------------------------------
// CSGSlice
//------------------------------------------------------------------------------

#ifndef SINGLE_THREADED

class CSGSlice {
  public:
    CSGSlice(int aSize, int aID)
    {
      mData = new Voxel[aSize];
      mID = aID;
      mDone = false;
    }

    ~CSGSlice()
    {
      delete[] mData;
    }

    inline Voxel * Data()
    {
      return mData;
    }

    inline int ID()
    {
      return mID;
    }

    inline bool IsDone()
    {
      return mDone;
    }

    inline void Done()
    {
      mDone = true;
    }

  private:
    Voxel * mData; ///< Slice data.
    int mID;       ///< Slice ID.
    bool mDone;    ///< Flag: true if the slice has been calculated.
};

#endif // !SINGLE_THREADED


//------------------------------------------------------------------------------
// CSGSlicePool
//------------------------------------------------------------------------------

#ifndef SINGLE_THREADED

class CSGSlicePool {
  public:
    CSGSlicePool()
    {
      mNextID = 0;
      mSampleSpace = 0;
      mCSGRoot = 0;
    }

    ~CSGSlicePool()
    {
      // If there are any slices left in the list, delete them now
      list<CSGSlice *>::iterator it;
      for(it = mSlices.begin(); it != mSlices.end(); ++ it)
        delete[] (CSGSlice *) *it;
    }

    /// Define the sample space to use.
    inline void SetSampleSpace(SampleSpace * aSampleSpace)
    {
      mSampleSpace = aSampleSpace;
    }

    /// Create a new slice and put it in the pool.
    CSGSlice * NewSlice()
    {
      if(!mSampleSpace)
        throw runtime_error("SampleSpace object undefined.");

      CSGSlice * result = 0;
      mMutex.lock();
      if(mNextID < mSampleSpace->mDiv[2])
      {
        int count = mSampleSpace->mDiv[0] * mSampleSpace->mDiv[1];
        result = new CSGSlice(count, mNextID);
        mSlices.push_back(result);
        ++ mNextID;
      }
      mMutex.unlock();
      return result;
    }

    /// Signal that the slice is done.
    void SliceDone(CSGSlice * aSlice)
    {
      mMutex.lock();
      aSlice->Done();
      mMutex.unlock();
      mCondition.notify_all();
    }

    /// Get a specific slice. The slice is removed from the list (NOTE: The
    /// caller is respondible for freeing the slice). This call is blocking,
    /// until the corresponding slice is ready.
    CSGSlice * GetSlice(int aID)
    {
      mMutex.lock();

      // Start by finding the slice (wait until it is in the queue)
      CSGSlice * result = 0;
      list<CSGSlice *>::iterator it;
      while(!result)
      {
        for(it = mSlices.begin(); it != mSlices.end(); ++ it)
        {
          if((*it)->ID() == aID)
          {
            result = *it;
            break;
          }
        }

        // It was not in the queue - wait for the queue to be updated
        if(!result)
          mCondition.wait(mMutex);
      }

      // Now, wait for the slice to be done (processed)
      while(!result->IsDone())
        mCondition.wait(mMutex);

      // Remove the slice from the list
      mSlices.erase(it);

      mMutex.unlock();

      return result;
    }

    /// Pointer to the CSG tree root node
    CSGNode * mCSGRoot;

  private:
    mutex mMutex;
    condition_variable mCondition;
    int mNextID;
    SampleSpace * mSampleSpace;
    list<CSGSlice *> mSlices;
};

#endif // !SINGLE_THREADED


//------------------------------------------------------------------------------
// Slice calculation thread function
//------------------------------------------------------------------------------

#ifndef SINGLE_THREADED

static void SliceCalcThread(void * aArg)
{
  CSGSlicePool * slicePool = (CSGSlicePool *) aArg;
  while(CSGSlice * slice = slicePool->NewSlice())
  {
    // Generate slice data
    slicePool->mCSGRoot->ComposeSlice(slice->Data(), slice->ID());
    slicePool->SliceDone(slice);
  }
}

#endif // !SINGLE_THREADED


//------------------------------------------------------------------------------
// CSGJob
//------------------------------------------------------------------------------

CSGJob::CSGJob()
{
  mCSGRoot = 0;
  mResolution = Vector3(0.1, 0.1, 0.1);
  mOutputFileName = string("output");
  mOutputType = otMesh;
  mOutputFormat = ofSTL;
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

  ImageWriter * imgOut = 0;
  Polygonize polygonize;
  if(mOutputType == otSlices)
  {
    // Prepare image writer
    if(mOutputFormat == ofTGA)
      imgOut = new TGAImageWriter;
    else
      throw runtime_error("Unsupported output format for slices.");
    imgOut->SetFormat(space.mDiv[0], space.mDiv[1], ImageWriter::pfSigned8);
  }
  else if(mOutputType == otMesh)
  {
    // Prepare polygonizer
    polygonize.SetSampleSpace(&space);
  }

  try
  {
    // Perform operation...
    cout << "Executing job..." << flush;
#ifdef SINGLE_THREADED
    double polygonizeTime = 0;
#endif
    mTimer.Push();

#ifndef SINGLE_THREADED
    // Start slice calculation threads
    CSGSlicePool slicePool;
    slicePool.SetSampleSpace(&space);
    slicePool.mCSGRoot = mCSGRoot;
    thread thread1(SliceCalcThread, (void *) &slicePool);
    thread thread2(SliceCalcThread, (void *) &slicePool);
    CSGSlice * slice, * sliceOld = 0;
#else
    // Allocate memory for two slices
    vector<Voxel> voxelSlice1, voxelSlice2;
    voxelSlice1.resize(space.mDiv[0] * space.mDiv[1]);
    voxelSlice2.resize(space.mDiv[0] * space.mDiv[1]);
    Voxel * slice = &voxelSlice1[0];
    Voxel * sliceOld = &voxelSlice2[0];
#endif

    for(int i = 0; i < space.mDiv[2]; ++ i)
    {
      // Get the next slice
#ifndef SINGLE_THREADED
      slice = slicePool.GetSlice(i);
#else
      mCSGRoot->ComposeSlice(slice, i);
#endif

      // Write slice image or genereate mesh triangles?
      if(mOutputType == otSlices)
      {
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
#ifndef SINGLE_THREADED
        imgOut->SetData(slice->Data());
#else
        imgOut->SetData(slice);
#endif
        imgOut->SetSliceNo(i);
        imgOut->SetSampleSpace(&space);
        imgOut->SaveToFile(name.str().c_str());
      }
      else if(mOutputType == otMesh)
      {
#ifndef SINGLE_THREADED
        if(i > 0)
          polygonize.AppendSlicePair(sliceOld->Data(), slice->Data(), i - 1);
#else
        mTimer.Push();
        if(i > 0)
          polygonize.AppendSlicePair(sliceOld, slice, i - 1);
        polygonizeTime += mTimer.PopDelta();
#endif
      }

      // Swap slice buffers
#ifndef SINGLE_THREADED
      if(sliceOld)
        delete sliceOld;
      sliceOld = slice;
#else
      Voxel * tmp = sliceOld;
      sliceOld = slice;
      slice = tmp;
#endif
    }

#ifdef SINGLE_THREADED
    if(sliceOld)
      delete sliceOld;
#endif

    dt = mTimer.PopDelta();
#ifndef SINGLE_THREADED
    cout << "done! (" << int(dt * 1000.0 + 0.5) << " ms)" << endl;
#else
    cout << "done! (" << int(dt * 1000.0 + 0.5) << " ms";
    if(polygonizeTime > 0.0)
      cout << ", polygonize: "  << int(polygonizeTime * 1000.0 + 0.5) << " ms)" << endl;
    else
      cout << ")" << endl;
#endif

    if(imgOut)
    {
      delete imgOut;
      imgOut = 0;
    }

    // Write mesh file
    if(mOutputType == otMesh)
    {
      cout << "Writing mesh file..." << flush;
      mTimer.Push();
      Mesh mesh;
      polygonize.ToMesh(mesh);
      if(mOutputFormat == ofSTL)
      {
        STLMeshWriter meshWriter;
        meshWriter.SetMesh(&mesh);
        meshWriter.SaveToFile((mOutputFileName + string(".stl")).c_str());
      }
      else
        throw runtime_error("Unsupported output format for meches.");
      dt = mTimer.PopDelta();
      cout << "done! (" << int(dt * 1000.0 + 0.5) << " ms)" << endl;
    }
  }
  catch(...)
  {
    if(imgOut)
      delete imgOut;
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
          else if(s == string("mesh"))
            mOutputType = otMesh;
          else
            cout << "  Warning: Unrecognized output type: " << s << endl;
        }

        // Get output format
        s = e->Attribute("format");
        if(s)
        {
          if(s == string("tga"))
            mOutputFormat = ofTGA;
          else if(s == string("stl"))
            mOutputFormat = ofSTL;
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
