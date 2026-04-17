#!/usr/bin/env python
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from parser import XMLParser
from dof_optimizer import DOFOptimizer

# Test 4
xml_path = "data/eg1_truss_temp.xml"
model = XMLParser(xml_path).parse()

print("Before DOFOptimizer:")
for n_id in sorted(model.nodes.keys()):
    print(f"  Node {n_id}: dofs = {model.nodes[n_id].dofs}")
print()

opt = DOFOptimizer(model)
num_eq, semi_bw, _ = opt.optimize()

print("After DOFOptimizer:")
for n_id in sorted(model.nodes.keys()):
    print(f"  Node {n_id}: dofs = {model.nodes[n_id].dofs}")
print()

print(f"num_eq = {num_eq}, semi_bw = {semi_bw}")
print()

# Check element DOF mappings
print("Element DOF mappings:")
for el_id, el in model.elements.items():
    if el.type == 'truss':
        element_dofs = el.node_i.dofs[0:2] + el.node_j.dofs[0:2]
    else:
        element_dofs = el.node_i.dofs + el.node_j.dofs
    
    print(f"  {el_id}: node_i={el.node_i.id}({el.node_i.dofs[0:2]}), node_j={el.node_j.id}({el.node_j.dofs[0:2]})")
    print(f"        element_dofs = {element_dofs}")
