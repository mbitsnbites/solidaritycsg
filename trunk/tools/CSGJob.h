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

#ifndef _CSGJOB_H_
#define _CSGJOB_H_

#include <string>
#include <tinyxml.h>
#include <SolidarityCSG.h>
#include "OS/OSTime.h"

/// CSG job class.
class CSGJob {
  public:
    /// Constructor.
    CSGJob();

    /// Destructor.
    ~CSGJob();

    /// Load job description from an XML file.
    void LoadFromXML(const char * aFileName);

    /// Execute job.
    void Execute();

  private:
    /// Output type enumerator.
    enum OutputType {
      otSlices = 1,
      otMesh = 2
    };

    /// Output format enumerator.
    enum OutputFormat {
      ofTGA = 1,
      ofSTL = 2
    };

    /// Load the settings description from an XML document node.
    void LoadSettings(TiXmlNode * aNode);

    /// Load the volume description from an XML document node.
    void LoadVolume(TiXmlNode * aNode);

    /// Recursively load a node in the volume description from an XML document
    /// element.
    csg::CSGNode * LoadCSGNode(TiXmlElement * aElement);

    /// CSG tree root node.
    csg::CSGNode * mCSGRoot;

    /// Resolution.
    csg::Vector3 mResolution;

    /// Output file name.
    std::string mOutputFileName;

    /// Output type.
    OutputType mOutputType;

    /// Output format.
    OutputFormat mOutputFormat;

    /// Timer object (for profiling etc).
    os::Timer mTimer;
};

#endif // _CSGJOB_H_
