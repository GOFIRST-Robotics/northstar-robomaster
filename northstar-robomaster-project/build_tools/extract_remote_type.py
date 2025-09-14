# Copyright (c) 2020-2021 NorthStart
#
# This file is part of NorthStarFleet2025.
#
# NorthStarFleet2025 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# NorthStarFleet2025 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with NorthStarFleet2025.  If not, see <https://www.gnu.org/licenses/>.

from SCons.Script import *

DEFAULT_REMOTE = "DJI_REMOTE"

VALID_REMOTE_TYPES   = [ "DJI_REMOTE", 
                        "FLY_SKY"]

def get_remote_type():
    remote_type = ARGUMENTS.get("remote")
    # Configure robot type and check against valid robot type
    # If there is no optional argument, revert back to the macro in robot_type.hpp
    if remote_type == None:
        remote_type = DEFAULT_REMOTE
    if remote_type not in VALID_REMOTE_TYPES:
        raise Exception("No valid remote type specified. check extract_remote_type.py for valid remote types")

    return remote_type
