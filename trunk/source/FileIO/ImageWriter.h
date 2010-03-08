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

#include "../Voxelize/Voxelize.h"

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
      mData = 0;
      mWidth = mHeight = 0;
      mPixelFormat = pfSigned8;
      mSampleSpace = 0;
      mSliceNo = 0;
    }

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

    /// Define the current slice number.
    inline void SetSliceNo(int aSliceNo)
    {
      mSliceNo = aSliceNo;
    }

    /// Define the image data.
    inline void SetData(void * aData)
    {
      mData = aData;
    }

    /// Save the image to a file.
    virtual void SaveToFile(const char * aFileName) = 0;

  protected:
    void * mData;               ///< Image pixel data.
    int mWidth;                 ///< Bitmap width.
    int mHeight;                ///< Bitmap height.
    PixelFormat mPixelFormat;   ///< Pixel format.
    SampleSpace * mSampleSpace; ///< Sample space definition
    int mSliceNo;               ///< Current slice number.
};

}

#endif // _IMAGEWRITER_H_
