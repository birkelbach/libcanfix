#  Copyright (c) 2021 Phil Birkelbach
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

include(CheckIncludeFile)

# This allows us to use VERSION in the project command
cmake_policy(SET CMP0048 NEW)
project(OpenDAX VERSION 0.7)
cmake_minimum_required(VERSION 3.10.2)

set(BUILD_SHARED_LIBS ON)


enable_testing()

set(CMAKE_BUILD_TYPE Debug)


# check_include_file(string.h HAVE_STRING_H)

# This is for the config.h file
include_directories(${PROJECT_BINARY_DIR}/src)
include_directories(src)

add_subdirectory(src)

add_subdirectory(tests)

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message("Building Debug Symbols and Verbose Warnings")
endif()
