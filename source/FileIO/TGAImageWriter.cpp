/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdexcept>
#include <fstream>
#include "TGAImageWriter.h"

using namespace std;

namespace csg {

//-----------------------------------------------------------------------------
// TGAImageWriter
//-----------------------------------------------------------------------------

void TGAImageWriter::SaveToFile(const char * aFileName)
{
  // Sanity check
  if(!mData || (mWidth < 1) || (mHeight < 1))
    throw runtime_error("Badly defined image.");

  // We only support gray scale images
  if(mPixelFormat != pfGray8)
    throw runtime_error("Unsupported pixel format.");

  // Open output file
  ofstream f(aFileName, ios_base::out | ios_base::binary);
  if(f.fail())
    throw runtime_error("Unable to write output file.");

  // Write header
  unsigned char header[18];
  header[0] = 0;  // ID length
  header[1] = 0;  // Color map type (0 = none)
  header[2] = 3;  // Image type (3 = gray)
  header[3] = 0;  // First color map entry (16 bits)
  header[4] = 0;  // -"-
  header[5] = 0;  // Number of color map entries (16 bits)
  header[6] = 0;  // -"-
  header[7] = 0;  // Bits per color map entry
  header[8] = 0;  // Image x origin (16 bits)
  header[9] = 0;  // -"-
  header[10] = 0;  // Image y origin (16 bits)
  header[11] = 0;  // -"-
  header[12] = mWidth & 255;         // Image width (16 bits)
  header[13] = (mWidth >> 8) & 255;  // -"-
  header[14] = mHeight & 255;        // Image width (16 bits)
  header[15] = (mHeight >> 8) & 255; // -"-
  header[16] = 8;  // Bits per pixel
  header[17] = 32; // Flip flag (bit 5: use top-to-bottom ordering)
  f.write((const char *) header, 18);

  // Write data
  f.write((const char *) mData, mWidth * mHeight);

  // Close output file
  f.close();
}


}
