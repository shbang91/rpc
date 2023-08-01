#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2022 Stéphane Caron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Standalone version of meshcat-shapes.

See <https://pypi.org/project/meshcat-shapes/>. We keep this copy in examples/
so that it can be used by examples that need it without making meshcat-shapes
(and thus meshcat) a dependency of the project.
"""

import meshcat
import numpy as np


def __attach_axes(
    handle: meshcat.Visualizer,
    length: float = 0.05,
    thickness: float = 0.002,
    opacity: float = 1.0,
) -> None:
    """Attach a set of three basis axes to a MeshCat handle.

    Args:
        handle: MeshCat handle to attach the basis axes to.
        length: Length of axis unit vectors.
        thickness: Thickness of axis unit vectors.
        opacity: Opacity of all three unit vectors.

    Note:
        As per the de-facto standard (Blender, OpenRAVE, RViz, ...), the
        x-axis is red, the y-axis is green and the z-axis is blue.
    """
    direction_names = ["x", "y", "z"]
    colors = [0xFF0000, 0x00FF00, 0x0000FF]
    rotation_axes = [[0, 0, 1], [0, 1, 0], [1, 0, 0]]
    position_cylinder_in_frame = 0.5 * length * np.eye(3)
    for i in range(3):
        dir_name = direction_names[i]
        material = meshcat.geometry.MeshLambertMaterial(
            color=colors[i], opacity=opacity
        )
        transform_cylinder_to_frame = meshcat.transformations.rotation_matrix(
            np.pi / 2, rotation_axes[i]
        )
        transform_cylinder_to_frame[0:3, 3] = position_cylinder_in_frame[i]
        cylinder = meshcat.geometry.Cylinder(length, thickness)
        handle[dir_name].set_object(cylinder, material)
        handle[dir_name].set_transform(transform_cylinder_to_frame)


def frame(
    handle: meshcat.Visualizer,
    axis_length: float = 0.1,
    axis_thickness: float = 0.005,
    opacity: float = 1.0,
    origin_color: int = 0x000000,
    origin_radius: float = 0.01,
) -> None:
    """Set MeshCat handle to a frame, represented by an origin and three axes.

    Args:
        handle: MeshCat handle to attach the frame to.
        axis_length: Length of axis unit vectors, in [m].
        axis_thickness: Thickness of axis unit vectors, in [m].
        opacity: Opacity of all three unit vectors.
        origin_color: Color of the origin sphere.
        origin_radius: Radius of the frame origin sphere, in [m].

    Note:
        As per the de-facto standard (Blender, OpenRAVE, RViz, ...), the
        x-axis is red, the y-axis is green and the z-axis is blue.
    """
    material = meshcat.geometry.MeshLambertMaterial(
        color=origin_color, opacity=opacity
    )
    sphere = meshcat.geometry.Sphere(origin_radius)
    handle.set_object(sphere, material)
    __attach_axes(handle, axis_length, axis_thickness, opacity)
