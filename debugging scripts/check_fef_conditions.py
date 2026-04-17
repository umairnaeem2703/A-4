#!/usr/bin/env python
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from parser import XMLParser
from element_physics import ElementPhysics
from dof_optimizer import DOFOptimizer

# Test 5
xml_path = "data/eg2_beam_temp.xml"
model = XMLParser(xml_path).parse()

opt = DOFOptimizer(model)
num_eq, semi_bw, _ = opt.optimize()

print("=== Element FEF Conditions ===\n")

for el_id, el in model.elements.items():
    physics = ElementPhysics(el)
    fef_cond = physics._determine_fef_condition(model)
    
    print(f"{el_id}: {el.node_i.id}->{el.node_j.id}")
    print(f"  release_start={el.release_start}, release_end={el.release_end}")
    print(f"  FEF condition: {fef_cond}")
    
    # Check supports
    sup_i = model.supports.get(el.node_i.id)
    sup_j = model.supports.get(el.node_j.id)
    
    if sup_i:
        print(f"  Support at node_i: ux={sup_i.restrain_ux}, uy={sup_i.restrain_uy}, rz={sup_i.restrain_rz}")
    else:
        print(f"  No support at node_i")
    
    if sup_j:
        print(f"  Support at node_j: ux={sup_j.restrain_ux}, uy={sup_j.restrain_uy}, rz={sup_j.restrain_rz}")
    else:
        print(f"  No support at node_j")
    
    # Check thermal FEF
    load_case = model.load_cases["LC1"]
    fef_local = physics.get_local_fef(load_case, model)
    
    print(f"  FEF local: [{fef_local[0][0]:.1f}, {fef_local[1][0]:.1f}, {fef_local[2][0]:.1f}, {fef_local[3][0]:.1f}, {fef_local[4][0]:.1f}, {fef_local[5][0]:.1f}]")
    print()
