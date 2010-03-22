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
#include <iostream>
#include <string>
#include <list>
#include "CSGJob.h"

using namespace std;


/// Main application entry
int main(int argc, char ** argv)
{
  // Collect arguments
  CSGJob::OperationMode mode = CSGJob::omMultiThreaded;
  list<string> jobFiles;
  for(int i = 1; i < argc; ++ i)
  {
    if(string(argv[i]) == string("-st"))
      mode = CSGJob::omSingleThreaded;
    else
      jobFiles.push_back(string(argv[i]));
  }

  // Check arguments
  if(jobFiles.size() < 1)
  {
    cout << "Usage: " << argv[0] << " [options] job-xml [job-xml [...]]" << endl;
    cout << "Options:" << endl;
    cout << " -st  Use single threaded operation (default is multi threaded operation)." << endl;
    return -1;
  }

  try
  {
    // Perform all the jobs
    for(list<string>::iterator i = jobFiles.begin(); i != jobFiles.end(); ++ i)
    {
      CSGJob job;
      job.LoadFromXML((*i).c_str());
      job.Execute(mode);
      cout << endl;
    }
  }
  catch(exception &e)
  {
    cout << "Error: " << e.what() << endl;
  }

  return 0;
}
