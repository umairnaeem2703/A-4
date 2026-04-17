#!/usr/bin/env python
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from parser import XMLParser, TemperatureL

# Load the model
xml_path = "data/eg1_truss_temp.xml"
model = XMLParser(xml_path).parse()

# Test thermal load FEF calculation directly
el_t1 = model.elements["T1"]
temp_load_t1 = model.load_cases["LC1"].loads[0]

print("=== T1 Element ===")
print(f"Area: {el_t1.section.A}")
print(f"Material.E: {el_t1.material.E}")
print(f"Material.alpha: {el_t1.material.alpha}")

print("\n=== Temperature Load ===")
print(f"Tu: {temp_load_t1.Tu}")
print(f"Tb: {temp_load_t1.Tb}")
print(f"T_uniform = (Tu + Tb) / 2 = ({temp_load_t1.Tu} + {temp_load_t1.Tb}) / 2 = {(temp_load_t1.Tu + temp_load_t1.Tb) / 2}")

# Calculate FEF manually
alpha = el_t1.material.alpha
E = el_t1.material.E
A = el_t1.section.A
T_uniform = (temp_load_t1.Tu + temp_load_t1.Tb) / 2.0

fef_0 = alpha * T_uniform * E * A
fef_3 = -alpha * T_uniform * E * A

print(f"\n=== Manual FEF Calculation ===")
print(f"fef[0] = alpha * T_uniform * E * A")
print(f"       = {alpha} * {T_uniform} * {E} * {A}")
print(f"       = {fef_0}")
print(f"fef[3] = -fef[0] = {fef_3}")

# Now call the method
fef_method = temp_load_t1.FEF("pin-pin", 8.0)  # L doesn't matter for thermal
print(f"\n=== FEF from Method ===")
print(f"fef[0] = {fef_method[0][0]}")
print(f"fef[1] = {fef_method[1][0]}")
print(f"fef[2] = {fef_method[2][0]}")
print(f"fef[3] = {fef_method[3][0]}")
