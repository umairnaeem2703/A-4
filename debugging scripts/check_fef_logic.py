#!/usr/bin/env python
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from parser import XMLParser, TemperatureL
from element_physics import ElementPhysics
from dof_optimizer import DOFOptimizer

# Test 4
xml_path = "data/eg1_truss_temp.xml"
model = XMLParser(xml_path).parse()

opt = DOFOptimizer(model)
num_eq, semi_bw, _ = opt.optimize()

load_case = model.load_cases["LC1"]

print("=== Load Types Check ===\n")

for i, load in enumerate(load_case.loads):
    print(f"Load {i}:")
    print(f"  Type: {type(load)}")
    print(f"  Class name: {load.__class__.__name__}")
    print(f"  Is TemperatureL: {isinstance(load, TemperatureL)}")
    print(f"  String comparison: {load.__class__.__name__ == 'TemperatureL'}")
print()

# Now check how get_local_fef processes them
el_t1 = model.elements["T1"]
physics = ElementPhysics(el_t1)

fef_cond_mech = physics._determine_fef_condition(model)
fef_cond_thermal = "fixed-fixed"

print(f"T1 Element:")
print(f"  Mechanical FEF condition: {fef_cond_mech}")
print(f"  Thermal FEF condition: {fef_cond_thermal}")
print()

fef_local = physics.get_local_fef(load_case, model)
print(f"T1 FEF local (computed by get_local_fef):")
print(f"  fef = [{fef_local[0][0]:.1f}, {fef_local[1][0]:.1f}, {fef_local[2][0]:.1f}, {fef_local[3][0]:.1f}]")
