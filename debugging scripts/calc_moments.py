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

load_case = model.load_cases["LC1"]

print("=== Manual Thermal Moment Calculation ===\n")

for el_id, el in model.elements.items():
    E = el.material.E
    alpha = el.material.alpha
    d = el.section.d
    I = el.section.I
    
    # Find the load  
    for load in load_case.loads:
        if load.element.id == el_id:
            Tu = load.Tu
            Tb = load.Tb
            T_grad = Tu - Tb
            T_uniform = (Tu + Tb) / 2.0
            
            # Base thermal moment
            M_thermal = (alpha * T_grad / d) * E * I
            
            print(f"{el_id}:")
            print(f"  T_grad = Tu - Tb = {Tu} - {Tb} = {T_grad}")
            print(f"  d = {d}, I = {I}, E = {E}, alpha = {alpha}")
            print(f"  M_thermal = {M_thermal}")
            
            # What should be the FEF?
            # For fixed-fixed: ±M_thermal
            # For pin-fixed: 0 at pin, + M_thermal at fixed (WRONG?)
            # Let me try: maybe the moment at fixed end should be 1.5 * M_thermal or 0.667 * M_thermal)
            
            physics = ElementPhysics(el)
            fef_cond = physics._determine_fef_condition(model)
            print(f"  FEF condition: {fef_cond}")
            
            fef_local = physics.get_local_fef(load_case, model)
            print(f"  FEF[2] (moment at i) = {fef_local[2][0]}")
            print(f"  FEF[5] (moment at j) = {fef_local[5][0]}")
            print()
