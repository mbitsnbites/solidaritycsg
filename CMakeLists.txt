#---------------------------------------------------------------------------------------------------
#  This file is part of SolidarityCSG.
#
#  SolidarityCSG is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  SolidarityCSG is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with SolidarityCSG.  If not, see <http://www.gnu.org/licenses/>.
#---------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.5)
project(SolidarityCSG)

# We build everything against the C++11 standard.
set(CMAKE_CXX_STANDARD 11)
set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MINGW)
  # For ease of deployment, statically link the standard libraries when using MinGW.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -static-libstdc++")

  # We target Windows Vista or higher.
  add_definitions(-D_WIN32_WINNT=0x0600)
endif()

if(MSVC)
  # Avoid CRT secure warnings when using MSVC.
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

add_subdirectory(source)
add_subdirectory(tools)

# Doxygen documentation.
find_package(Doxygen)
if(DOXYGEN_FOUND AND NOT (${CMAKE_VERSION} VERSION_LESS "3.9.0"))
  set(DOXYGEN_PROJECT_NAME SolidarityCSG)
  set(DOXYGEN_OUTPUT_DIRECTORY doc)
  set(DOXYGEN_GENERATE_HTML YES)
  set(DOXYGEN_GENERATE_LATEX NO)
  set(DOXYGEN_EXCLUDE third_party)
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_EXTRACT_STATIC YES)
  set(DOXYGEN_QUIET YES)
  set(DOXYGEN_WARN_IF_UNDOCUMENTED NO)
  doxygen_add_docs(doc
                   ${PROJECT_SOURCE_DIR}/source
                   COMMENT "Generating documentation with Doxygen")
endif()

