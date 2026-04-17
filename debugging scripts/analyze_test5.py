#!/usr/bin/env python
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from parser import XMLParser
from dof_optimizer import DOFOptimizer
from matrix_assembly import MatrixAssembler
from banded_solver import BandedSolver
from post_processor import PostProcessor

# Test 5: eg2_beam_temp
xml_path = "data/eg2_beam_temp.xml"
model = XMLParser(xml_path).parse()

print("=== Test 5: Beam with Thermal Gradient ===\n")

print("Model Info:")
print(f"Nodes: {list(model.nodes.keys())}")
print(f"Elements: {list(model.elements.keys())}")
print()

print("Element Details:")
for el_id, el in model.elements.items():
    print(f"  {el_id}: {el.node_i.id}->{el.node_j.id}, section.d={el.section.d}, section.I={el.section.I}")
print()

load_case = model.load_cases["LC1"]
print(f"Load case loads: {len(load_case.loads)}")
for i, load in enumerate(load_case.loads):
    if hasattr(load, 'Tu'):
        print(f"  Load {i}: {load.__class__.__name__} on {load.element.id}, Tu={load.Tu}, Tb={load.Tb}")
print()

opt = DOFOptimizer(model)
num_eq, semi_bw, _ = opt.optimize()

print(f"DOF assignment:")
for n_id in sorted(model.nodes.keys()):
    print(f"  Node {n_id}: dofs = {model.nodes[n_id].dofs}")
print()

assembler = MatrixAssembler(model, num_eq, semi_bw)
K_banded, F_global = assembler.assemble("LC1")

print(f"Assembled F_global:")
for i in range(len(F_global)):
    print(f"  F_global[{i}][0] = {F_global[i][0]:.6f}")
print()

print(f"Expected F_global:")
print(f"  F_global[n1.dofs[2]][0] = 12.0 (but got {F_global[0][0]:.6f} for DOF 0, which is n1.x)")
n1 = model.nodes[1]
print(f"  n1.dofs = {n1.dofs}")
if len(n1.dofs) > 2 and n1.dofs[2] >= 0:
    print(f"  F_global[n1.dofs[2]][0] = {F_global[n1.dofs[2]][0]:.6f}")
print()

solver = BandedSolver(K_banded, F_global, semi_bw)
D_active = solver.solve()

processor = PostProcessor(model, D_active, "LC1")

print(f"Displacements:")
for n_id in sorted(processor.displacements.keys()):
    disp = processor.displacements[n_id]
    print(f"  Node {n_id}: u={disp[0]:.6e}, v={disp[1]:.6e}, r={disp[2]:.6e}")
