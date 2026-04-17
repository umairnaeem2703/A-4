# Bug Fix Summary

## Problem
Two regression tests were failing:
- **test_regression_04**: F_global[1] was -21.50 instead of expected -32.24 (10.73 kN discrepancy)
- **test_regression_05**: F_global[n1.rz] was 14.14 instead of expected 12.0 (2.14 kNmm discrepancy)

All other values matched perfectly, including:
- Displacements were completely correct
- Some reaction force components matched expected values

This indicated the problem was not with the assembly logic or input data validation, but with how the load vector was being handled.

## Root Cause
The `BandedSolver` class was **modifying the F_global array in place** during Gaussian elimination. Specifically:

```python
# In banded_solver.py __init__:
self.F = F_global  # Direct reference - NOT a copy!

# In solve() method:
self.F[i][0] -= multiplier * self.F[k][0]  # Modifies the original F_global
```

This caused:
1. **Assembler** correctly computed F_global = [-72.08, -32.24]
2. **BandedSolver** modified this to [-72.08, -21.50] during forward elimination
3. **Test** checked the modified F_global, not the assembled one

## Solution
Made a deep copy of F_global in BandedSolver initialization:

```python
import copy

class BandedSolver:
    def __init__(self, K_banded: list, F_global: list, semi_bandwidth: int):
        self.K = K_banded
        self.F = copy.deepcopy(F_global)  # Create a copy, don't modify the original
        self.semi_bw = semi_bandwidth
        self.num_eq = len(K_banded)
```

## Impact
- ✅ **All 5 regression tests** now pass
- ✅ **All 5 unit tests** still pass  
- ✅ **All 2 interface tests** still pass
- ✅ Original assembled F_global is preserved for verification
- ✅ Solver still works correctly with its internal copy

## Files Changed
- **src/banded_solver.py**: Added `import copy` and made F_global a deep copy in __init__

## Verification
The assembled load vectors are now preserved and match exact structural mechanics calculations:
- Test 4 (3-Truss Thermal): F_global = [-72.08, -32.24] kN ✓
- Test 5 (2-Span Beam Thermal): F_global at nodes 1,2 = [12.0, -6.0] kNmm ✓

All displacements continue to match expectations perfectly.
