#!/usr/bin/env python
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from parser import XMLParser
from dof_optimizer import DOFOptimizer
from matrix_assembly import MatrixAssembler
from element_physics import ElementPhysics

# Load the truss temperature test model
xml_path = "data/eg1_truss_temp.xml"
model = XMLParser(xml_path).parse()

print("=== Model Loaded ===")
print(f"Elements: {list(model.elements.keys())}")
print(f"Nodes: {list(model.nodes.keys())}")
print(f"Load cases: {list(model.load_cases.keys())}")

load_case = model.load_cases["LC1"]
print(f"\nLoad case LC1 has {len(load_case.loads)} loads")
for i, load in enumerate(load_case.loads):
    print(f"  Load {i}: {load.__class__.__name__} on element {load.element.id if hasattr(load, 'element') else 'N/A'}")

# Assemble
opt = DOFOptimizer(model)
num_eq, semi_bw, _ = opt.optimize()
print(f"\nDOF Info: num_eq={num_eq}, semi_bw={semi_bw}")

assembler = MatrixAssembler(model, num_eq, semi_bw)
K_banded, F_global = assembler.assemble("LC1")

print(f"\nAssembled F_global:")
for i in range(min(6, len(F_global))):
    print(f"  F_global[{i}] = {F_global[i][0]}")

n1 = model.nodes[1]
print(f"\nNode 1 DOFs: {n1.dofs}")
print(f"F_global[n1.dofs[0]][0] = {F_global[n1.dofs[0]][0]}")
print(f"F_global[n1.dofs[1]][0] = {F_global[n1.dofs[1]][0]}")

# Debug individual elements
print("\n=== Per-Element Analysis ===")
for el_id, el in model.elements.items():
    physics = ElementPhysics(el)
    fef_local = physics.get_local_fef(load_case, model)
    k_local = physics.get_local_k()
    k_cond, fef_cond = physics.condense(k_local, fef_local)
    fef_global, _ = physics.transform_to_global(k_cond, fef_cond)
    
    print(f"\n{el_id}: {el.node_i.id} -> {el.node_j.id}")
    print(f"  c={physics.cos_x:.3f}, s={physics.sin_x:.3f}")
    print(f"  FEF local: [{fef_local[0][0]:.2f}, {fef_local[1][0]:.2f}, {fef_local[2][0]:.2f}, {fef_local[3][0]:.2f}]")
    print(f"  FEF global: [{fef_global[0][0]:.2f}, {fef_global[1][0]:.2f}, {fef_global[2][0]:.2f}, {fef_global[3][0]:.2f}]")
