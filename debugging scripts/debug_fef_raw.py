#!/usr/bin/env python
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from parser import XMLParser
import math_utils

# Load the model
xml_path = "data/eg1_truss_temp.xml"
model = XMLParser(xml_path).parse()

# Get the temperature loads
load_case = model.load_cases["LC1"]
temp_load_t1 = load_case.loads[0]  # T1 thermal load
temp_load_t2 = load_case.loads[1]  # T2 thermal load
temp_load_t3 = load_case.loads[2]  # T3 thermal load

print("=== FEF Values from TemperatureL.FEF() ===\n")

# Call FEF on T1
fef_t1_raw = temp_load_t1.FEF("pin-pin", 8.0)
print(f"T1 FEF (raw from TemperatureL):")
print(f"  fef is {len(fef_t1_raw)} rows x {len(fef_t1_raw[0]) if fef_t1_raw else 0} cols")
for i in range(len(fef_t1_raw)):
    print(f"  fef[{i}] = {fef_t1_raw[i][0]}")
print()

# Now test what happens when we add this to a 4x1 vector
fef_total = math_utils.zeros(4, 1)
print(f"fef_total (4x1): {[fef_total[i][0] for i in range(len(fef_total))]}")
print(f"Adding fef_t1_raw (6x1)...")
fef_total = math_utils.add(fef_total, fef_t1_raw)
print(f"Result: {[fef_total[i][0] for i in range(len(fef_total))]}")
print()

# What about when the first FEF comes from a different load?
fef_total = math_utils.zeros(4, 1)
fef_total = math_utils.add(fef_total, temp_load_t1.FEF("pin-pin", 8.0))
fef_total = math_utils.add(fef_total, temp_load_t2.FEF("pin-pin", 5.0))
fef_total = math_utils.add(fef_total, temp_load_t3.FEF("pin-pin", 6.0))

print(f"Combined FEF_total (after all 3 members):")
print(f"  Shape: {len(fef_total)} x {len(fef_total[0]) if fef_total else 0}")
for i in range(len(fef_total)):
    print(f"  fef_total[{i}] = {fef_total[i][0]}")
