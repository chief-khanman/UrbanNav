"""Vendored pure-Python port of the RVO2 / ORCA collision-avoidance library.

Source:  https://github.com/chengji253/RVO2-python  (MIT License)
Commit:  57b164cee743ec7d2ab0537045903a8b66afa537
Files:   Vector2.py, Line.py, Obstacle.py, Agent.py, KdTree.py, RVOSimulator.py
         (the demo scenarios blocks.py / roadmap.py / Circle.py / plotPic.py are
         not vendored).

The only modification from upstream is turning the flat sibling imports
(``from Vector2 import *``) into package-relative imports
(``from .Vector2 import *``) so the code is importable as ``urbannav.rvo2_python``.
Keep the rest byte-identical to upstream so it can be re-synced by diffing; the
UrbanNav-facing adapter lives in ``urbannav.controller_orca``.

This replaces the previously vendored C++/Cython ``Python-RVO2`` package
(https://github.com/sybrenstuvel/Python-RVO2), which needed a CMake build step.

Algorithm reference:
    J. van den Berg, S. J. Guy, M. Lin, D. Manocha, "Reciprocal n-Body Collision
    Avoidance", Robotics Research (ISRR 2009), Springer Tracts in Advanced
    Robotics vol. 70, 2011. https://doi.org/10.1007/978-3-642-19457-3_1
    Official C++ implementation: https://github.com/snape/RVO2
"""
from .RVOSimulator import RVOSimulator
from .Vector2 import Vector2

__all__ = ['RVOSimulator', 'Vector2']
