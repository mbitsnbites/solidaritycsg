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
#include <SolidarityCSG.h>
#include <tinyxml.h>

using namespace csg;
using namespace std;


/// CSG job class.
class CSGJob {
  public:
    /// Load the settings description from an XML document node.
    void LoadSettings(TiXmlNode * aNode)
    {
      cout << " Reading settings..." << endl;
      // FIXME
    }

    /// Load the volume description from an XML document node.
    void LoadVolume(TiXmlNode * aNode)
    {
      cout << " Reading volume description..." << endl;
      // FIXME
    }
};


/// Main application entry
int main(int argc, char ** argv)
{
  // Check/get arguments
  if(argc != 2)
  {
    cout << "Usage: " << argv[0] << " job-xml" << endl;
    return -1;
  }
  char * inFile = argv[1];

  try
  {
    CSGJob job;

    // Load input XML
    cout << "Loading job description " << inFile << "..." << endl;
    TiXmlDocument xmlDoc;
    if(!xmlDoc.LoadFile(inFile))
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
          job.LoadSettings(node);

        // Volume description node?
        else if(node->Value() == string("volume"))
          job.LoadVolume(node);

        // Unrecognized node?
        else
          cout << " Warning: Unrecognized node: " << node->Value() << endl;
      }

      // Next node...
      node = node->NextSibling();
    }

    // Perform operation...
    cout << "Executing job" << inFile << "..." << flush;
    // FIXME
    cout << "done!" << endl;
  }
  catch(exception &e)
  {
    cout << "Error: " << e.what() << endl;
  }

  return 0;
}
