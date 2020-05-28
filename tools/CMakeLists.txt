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

set(TINYXML_SRCS
    TinyXML/tinyxml.cpp
    TinyXML/tinyxmlerror.cpp
    TinyXML/tinyxmlparser.cpp
    TinyXML/tinystr.cpp
    )
add_library(tinyxml ${TINYXML_SRCS})
target_include_directories(tinyxml PUBLIC TinyXML)

set(TINYTHREAD_SRCS
    TinyThread/tinythread.cpp
    )
add_library(tinythread ${TINYTHREAD_SRCS})
target_include_directories(tinythread PUBLIC TinyThread)
find_package(Threads REQUIRED)
target_link_libraries(tinythread Threads::Threads)

set(CSG_SRCS
    csg.cpp
    CSGJob.cpp
    OS/OSTime.cpp
    )
add_executable(csg ${CSG_SRCS})
target_link_libraries(csg solidaritycsg tinyxml tinythread)

set(DEMO_SRCS
    demo.cpp
    )
add_executable(demo ${DEMO_SRCS})
target_link_libraries(demo solidaritycsg)

