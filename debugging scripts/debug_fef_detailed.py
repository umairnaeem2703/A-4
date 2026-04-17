#!/usr/bin/env python
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from parser import XMLParser
from dof_optimizer import DOFOptimizer
from matrix_assembly import MatrixAssembler
from element_physics import ElementPhysics
import math_utils

# Load the model
xml_path = "data/eg1_truss_temp.xml"
model = XMLParser(xml_path).parse()

load_case = model.load_cases["LC1"]

print("=== Detailed Per-Element FEF Analysis ===\n")

totalX = 0
totalY = 0

for el_id, el in model.elements.items():
    physics = ElementPhysics(el)
    L = physics.L
    
    # Get local FEF
    fef_local = physics.get_local_fef(load_case, model)
    
    print(f"{el_id}: Node {el.node_i.id}->{el.node_j.id}, L={L:.3f}, angle={abs(physics.sin_x):.3f} (sin)")
    print(f"  Coordinates: ({el.node_i.x}, {el.node_i.y}) -> ({el.node_j.x}, {el.node_j.y})")
    print(f"  cos_x={physics.cos_x:.3f}, sin_x={physics.sin_x:.3f}")
    print(f"  Local FEF[0]={fef_local[0][0]:.1f}, FEF[3]={fef_local[3][0]:.1f}")
    
    # Transform to global using standard transformation
    c, s = physics.cos_x, physics.sin_x
    
    # Global components for FEF at node i (first 2 rows)
    fef_i_x = c * fef_local[0][0] - s * fef_local[1][0]
    fef_i_y = s * fef_local[0][0] + c * fef_local[1][0]
    
    # Global components for FEF at node j (second 2 rows)
    fef_j_x = c * fef_local[2][0] - s * fef_local[3][0]
    fef_j_y = s * fef_local[2][0] + c * fef_local[3][0]
    
    print(f"  Global FEF at node {el.node_i.id}: Fx={fef_i_x:.1f}, Fy={fef_i_y:.1f}")
    print(f"  Global FEF at node {el.node_j.id}: Fx={fef_j_x:.1f}, Fy={fef_j_y:.1f}")
    
    # Only node 1 is free, so accumulate its values
    if el.node_i.id == 1:
        totalX += fef_i_x
        totalY += fef_i_y
    if el.node_j.id == 1:
        totalX += fef_j_x
        totalY += fef_j_y
    print()

print(f"Total FEF contributions to node 1:")
print(f"  Fx (local) = {totalX:.1f}")
print(f"  Fy (local) = {totalY:.1f}")
print(f"\nAfter assembly (F_global -= fef):")
print(f"  F_global[0] = 0 - {totalX:.1f} = {-totalX:.1f}")
print(f"  F_global[1] = 0 - {totalY:.1f} = {-totalY:.1f}")
print(f"\nExpected from test:")
print(f"  F_global[0] = -72.08")
print(f"  F_global[1] = -32.23")
