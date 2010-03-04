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

#ifndef _IMAGEWRITER_H_
#define _IMAGEWRITER_H_

namespace csg {

/// Image writer base class.
class ImageWriter {
  public:
    enum PixelFormat {
      pfGray8 = 1
    };

    /// Constructor.
    ImageWriter()
    {
      mData = 0;
      mWidth = mHeight = 0;
      mPixelFormat = pfGray8;
    }

    /// Define the image format.
    inline void SetFormat(int aWidth, int aHeight, PixelFormat aPixelFormat)
    {
      mWidth = aWidth;
      mHeight = aHeight;
      mPixelFormat = aPixelFormat;
    }

    /// Define the image data.
    inline void SetData(void * aData)
    {
      mData = aData;
    }

    /// Save the image to a file.
    virtual void SaveToFile(const char * aFileName) = 0;

  protected:
    void * mData;              ///< Image pixel data.
    int mWidth;                ///< Bitmap width.
    int mHeight;               ///< Bitmap height.
    PixelFormat mPixelFormat;  ///< Pixel format.
};

}

#endif // _VOXELIZE_H_
