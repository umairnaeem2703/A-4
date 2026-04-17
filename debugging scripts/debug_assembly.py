#!/usr/bin/env python
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from parser import XMLParser
from dof_optimizer import DOFOptimizer
from matrix_assembly import MatrixAssembler
from element_physics import ElementPhysics
import math_utils

xml_path = "data/eg1_truss_temp.xml"
model = XMLParser(xml_path).parse()

print("=== Node DOF Assignment ===")
for n_id in sorted(model.nodes.keys()):
    node = model.nodes[n_id]
    print(f"Node {n_id}: dofs={node.dofs}")

opt = DOFOptimizer(model)
num_eq, semi_bw, _ = opt.optimize()
print(f"\nAfter optimization: num_eq={num_eq}, semi_bw={semi_bw}")

print("\n=== After DOFOptimizer ===")
for n_id in sorted(model.nodes.keys()):
    node = model.nodes[n_id]
    print(f"Node {n_id}: dofs={node.dofs}")

# Check element DOF assignments
print("\n=== Element DOF Analysis ===")
for el_id, el in model.elements.items():
    if el.type == 'truss':
        element_dofs = el.node_i.dofs[0:2] + el.node_j.dofs[0:2]
    else:
        element_dofs = el.node_i.dofs + el.node_j.dofs
    
    print(f"{el_id}: {el.node_i.id}->{el.node_j.id}")
    print(f"  node_i.dofs[0:2] = {el.node_i.dofs[0:2]}")
    print(f"  node_j.dofs[0:2] = {el.node_j.dofs[0:2]}")
    print(f"  element_dofs = {element_dofs}")

# Now assemble and check
assembler = MatrixAssembler(model, num_eq, semi_bw)
load_case = model.load_cases["LC1"]

# Manual assembly with print statements
K_banded = math_utils.zeros(num_eq, semi_bw)
F_global = math_utils.zeros(num_eq, 1)

print("\n=== Manual Assembly with Logging ===")
for el_id, el in model.elements.items():
    physics = ElementPhysics(el)
    k_local = physics.get_local_k()
    fef_local = physics.get_local_fef(load_case, model)
    
    k_condensed, fef_condensed = physics.condense(k_local, fef_local)
    k_global, fef_global = physics.transform_to_global(k_condensed, fef_condensed)
    
    if el.type == 'truss':
        element_dofs = el.node_i.dofs[0:2] + el.node_j.dofs[0:2]
    else:
        element_dofs = el.node_i.dofs + el.node_j.dofs
    
    print(f"\n{el_id}:")
    print(f"  FEF global: [{fef_global[0][0]:.1f}, {fef_global[1][0]:.1f}, {fef_global[2][0]:.1f}, {fef_global[3][0]:.1f}]")
    print(f"  element_dofs: {element_dofs}")
    
    for row_idx in range(len(element_dofs)):
        dof_row = element_dofs[row_idx]
        if dof_row >= 0:
            print(f"    F_global[{dof_row}] -= {fef_global[row_idx][0]:.1f}")
            F_global[dof_row][0] -= fef_global[row_idx][0]
            print(f"      => F_global[{dof_row}] = {F_global[dof_row][0]:.1f}")

print(f"\n=== Final Assembly ===")
print(f"F_global[0] = {F_global[0][0]:.2f}")
print(f"F_global[1] = {F_global[1][0]:.2f}")
print(f"\nExpected:")
print(f"F_global[0] = -72.08")
print(f"F_global[1] = -32.23")
