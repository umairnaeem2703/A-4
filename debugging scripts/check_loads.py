#!/usr/bin/env python
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from parser import XMLParser

# Load the test model
xml_path = "data/eg1_truss_temp.xml"
model = XMLParser(xml_path).parse()

load_case = model.load_cases["LC1"]
print(f"Load Case: {load_case.id} ({load_case.name})")
print(f"Total loads: {len(load_case.loads)}")
print()

for i, load in enumerate(load_case.loads):
    print(f"Load {i}: {load.__class__.__name__}")
    if hasattr(load, 'node'):
        print(f"  Node: {load.node.id}")
        print(f"  Forces: fx={load.fx}, fy={load.fy}, mz={load.mz}")
    elif hasattr(load, 'element'):
        print(f"  Element: {load.element.id}")
        if hasattr(load, 'Tu'):
            print(f"  Tu={load.Tu}, Tb={load.Tb}")
    print()

print("Elements:")
for el_id, el in model.elements.items():
    print(f"  {el_id}: {el.type} ({el.node_i.id}->{el.node_j.id})")
    print(f"    A={el.section.A}, E={el.material.E}, alpha={el.material.alpha}")

print("\nSupports:")
for n_id, sup in model.supports.items():
    print(f"  Node {n_id}: ux={sup.restrain_ux}, uy={sup.restrain_uy}, rz={sup.restrain_rz}")
