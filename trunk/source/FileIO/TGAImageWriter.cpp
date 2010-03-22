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
#include <fstream>
#include <cstring>
#include <vector>
#include "TGAImageWriter.h"

using namespace std;

namespace csg {

// TGA header template definition (replace selected fields at run time)
static const unsigned char gTGAHeaderTemplate[18] = {
  0,  // ID length
  0,  // Color map type (0 = none)
  3,  // Image type (3 = gray)
  0,  // First color map entry (16 bits)
  0,  // -"-
  0,  // Number of color map entries (16 bits)
  0,  // -"-
  0,  // Bits per color map entry
  0,  // Image x origin (16 bits)
  0,  // -"-
  0,  // Image y origin (16 bits)
  0,  // -"-
  0,  // Image width (16 bits)
  0,  // -"-
  0,  // Image width (16 bits)
  0,  // -"-
  8,  // Bits per pixel
  32  // Flip flag (bit 5: use top-to-bottom ordering)
};


//-----------------------------------------------------------------------------
// TGAImageWriter
//-----------------------------------------------------------------------------

void TGAImageWriter::SaveToFile(const char * aFileName, void * aData,
  int aSliceNo)
{
  // Sanity check
  if(!aData || (mWidth < 1) || (mHeight < 1))
    throw runtime_error("Badly defined image.");

  // We only support 8-bit signed images
  if(mPixelFormat != pfSigned8)
    throw runtime_error("Unsupported pixel format.");

  // Open output file
  ofstream f(aFileName, ios_base::out | ios_base::binary);
  if(f.fail())
    throw runtime_error("Unable to write output file.");

  // Write header
  vector<unsigned char> header(18);
  memcpy(&header[0], gTGAHeaderTemplate, 18);
  header[12] = mWidth & 255;         // Image width (16 bits)
  header[13] = (mWidth >> 8) & 255;  // -"-
  header[14] = mHeight & 255;        // Image width (16 bits)
  header[15] = (mHeight >> 8) & 255; // -"-
  f.write((const char *) &header[0], 18);

  // Convert the image to the output format
  int imgSize = mWidth * mHeight;
  vector<unsigned char> data(imgSize);
  char * src = (char *)aData;
  unsigned char * dst = &data[0];
  for(int i = 0; i < imgSize; ++ i)
    *dst ++ = ((unsigned char)(*src ++)) + 128;

  // Write data
  f.write((const char *) &data[0], imgSize);

  // Close output file
  f.close();
}


}
