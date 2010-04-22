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

#ifndef _IMAGEWRITER_H_
#define _IMAGEWRITER_H_

#include "../SampleSpace.h"

namespace csg {

/// Image writer base class.
class ImageWriter {
  public:
    enum PixelFormat {
      pfSigned8 = 1
    };

    /// Constructor.
    ImageWriter()
    {
      mWidth = mHeight = 0;
      mPixelFormat = pfSigned8;
      mSampleSpace = 0;
    }

    /// Destructor.
    virtual ~ImageWriter() {}

    /// Define the image format.
    inline void SetFormat(int aWidth, int aHeight, PixelFormat aPixelFormat)
    {
      mWidth = aWidth;
      mHeight = aHeight;
      mPixelFormat = aPixelFormat;
    }

    /// Define the sample space (dimensions).
    inline void SetSampleSpace(SampleSpace * aSampleSpace)
    {
      mSampleSpace = aSampleSpace;
    }

    /// Save the image to a file.
    virtual void SaveToFile(const char * aFileName, void * aData, int aSliceNo = 0) = 0;

  protected:
    int mWidth;                 ///< Bitmap width.
    int mHeight;                ///< Bitmap height.
    PixelFormat mPixelFormat;   ///< Pixel format.
    SampleSpace * mSampleSpace; ///< Sample space definition
};

}

#endif // _IMAGEWRITER_H_
