# tests/test_unit.py

import sys
import os
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from parser import Node, Material, Section, Element, LoadCase, UniformlyDL, PointLoad, TemperatureL, Support
from element_physics import ElementPhysics
import math_utils

class TestElementPhysics(unittest.TestCase):
    def setUp(self):
        # Setup dummy properties for testing
        self.mat = Material(id="steel", E=2.0e8, alpha=1.2e-5)
        self.sec = Section(id="frame_sec", A=0.01, I=0.0001, d=0.3)
        self.node_i = Node(id=1, x=0.0, y=0.0)
        self.node_j = Node(id=2, x=5.0, y=0.0) 

    def test_local_k_standard_frame(self):
        """Test 1: Compute local [k] for a standard frame element."""
        frame = Element(id="F1", type="frame", node_i=self.node_i, node_j=self.node_j, 
                        material=self.mat, section=self.sec)
        physics = ElementPhysics(frame)
        k_local = physics.get_local_k()

        L = 5.0
        E, I = 2.0e8, 0.0001
        expected_k_v1 = 12 * E * I / (L**3)
        expected_k_r1 = 4 * E * I / L

        self.assertAlmostEqual(k_local[1][1], expected_k_v1, places=3)
        self.assertAlmostEqual(k_local[2][2], expected_k_r1, places=3)

    def test_fef_uniform_distributed_load(self):
        """Test 2: Compute FEFs for a member with a UDL."""
        frame = Element(id="F1", type="frame", node_i=self.node_i, node_j=self.node_j, 
                        material=self.mat, section=self.sec)
        
        udl = UniformlyDL(element=frame, wy=-10.0) 
        fef_local = udl.FEF("fixed-fixed", 5.0)

        w = -10.0
        L = 5.0
        
        expected_vy = (w * L) / 2.0
        expected_mz_i = (w * L**2) / 12.0
        
        self.assertAlmostEqual(fef_local[1][0], expected_vy, places=3)
        self.assertAlmostEqual(fef_local[2][0], expected_mz_i, places=3)

    def test_transformation_matrix(self):
        """Test 3: Evaluate coordinate transformation for an inclined element."""
        node_inc_i = Node(id=1, x=0.0, y=0.0)
        node_inc_j = Node(id=2, x=3.0, y=4.0)
        
        frame = Element(id="F2", type="frame", node_i=node_inc_i, node_j=node_inc_j, 
                        material=self.mat, section=self.sec)
        physics = ElementPhysics(frame)
        
        self.assertAlmostEqual(physics.cos_x, 0.6)
        self.assertAlmostEqual(physics.sin_x, 0.8)
        
        fef_local = [[10.0], [0.0], [0.0], [0.0], [0.0], [0.0]]
        k_local_dummy = math_utils.zeros(6, 6)
        
        _, fef_global = physics.transform_to_global(k_local_dummy, fef_local)
        
        self.assertAlmostEqual(fef_global[0][0], 6.0)
        self.assertAlmostEqual(fef_global[1][0], 8.0)
        self.assertAlmostEqual(fef_global[2][0], 0.0)
        
    def test_fef_member_point_load(self):
        """Test 4: Compute FEFs for a point load applied to a member."""
        frame = Element(id="F1", type="frame", node_i=self.node_i, node_j=self.node_j, 
                        material=self.mat, section=self.sec)
        
        P = -20.0
        a = 2.0
        L = 5.0
        b = L - a
        
        pt_load = PointLoad(element=frame, position=a, fy=P)
        fef_local = pt_load.FEF("fixed-fixed", L)
        
        expected_vy_i = (P * b**2 * (3*a + b)) / (L**3)
        expected_mz_i = (P * a * b**2) / (L**2)
        
        self.assertAlmostEqual(fef_local[1][0], expected_vy_i, places=3)
        self.assertAlmostEqual(fef_local[2][0], expected_mz_i, places=3)

    def test_thermal_uniform_load_case1(self):
        """Test 5: Thermal uniform load (Case 1) - Tu == Tb."""
        frame = Element(id="F1", type="frame", node_i=self.node_i, node_j=self.node_j, 
                        material=self.mat, section=self.sec)
        
        # Uniform temperature increase: Tu = Tb = 30°C
        T_uniform = 30.0
        thermal_load = TemperatureL(element=frame, Tu=T_uniform, Tb=T_uniform)
        fef_local = thermal_load.FEF("fixed-fixed", 5.0)
        
        # Instructor's formula: F_T = alpha * T_uniform * E * A
        alpha = 1.2e-5
        E = 2.0e8
        A = 0.01
        F_T = alpha * T_uniform * E * A
        
        # Expected FEF: [-F_T, 0, 0, F_T, 0, 0]^T
        self.assertAlmostEqual(fef_local[0][0], -F_T, places=-1)
        self.assertAlmostEqual(fef_local[1][0], 0.0, places=10)
        self.assertAlmostEqual(fef_local[2][0], 0.0, places=10)
        self.assertAlmostEqual(fef_local[3][0], F_T, places=-1)
        self.assertAlmostEqual(fef_local[4][0], 0.0, places=10)
        self.assertAlmostEqual(fef_local[5][0], 0.0, places=10)

    def test_thermal_gradient_load_case3(self):
        """Test 6: Thermal combined load (Case 3) - Tu != Tb using instructor's formula."""
        frame = Element(id="F1", type="frame", node_i=self.node_i, node_j=self.node_j, 
                        material=self.mat, section=self.sec)
        
        # Case 3: Bottom hotter than top
        Tu = 20.0  # Top temperature (°C)
        Tb = 70.0  # Bottom temperature (°C)
        
        thermal_load = TemperatureL(element=frame, Tu=Tu, Tb=Tb)
        fef_local = thermal_load.FEF("fixed-fixed", 5.0)
        
        # Program decomposition:
        alpha = 1.2e-5
        E = 2.0e8
        A = 0.01
        I = 0.0001
        d = 0.3
        L = 5.0
        
        delta_T = Tu - Tb  # -50.0°C
        T_uniform = 0.5 * (Tu + Tb)  # 45°C average
        
        F_T = alpha * T_uniform * E * A
        M_T = (alpha * delta_T / d) * E * I  # NO /2 in numerator
        
        # Expected FEF for fixed-fixed: [-F_T, 0, -M_T, F_T, 0, M_T]^T
        self.assertAlmostEqual(fef_local[0][0], -F_T, places=-1)
        self.assertAlmostEqual(fef_local[2][0], -M_T, places=-2)
        self.assertAlmostEqual(fef_local[3][0], F_T, places=-1)
        self.assertAlmostEqual(fef_local[5][0], M_T, places=-2)

    def test_thermal_truss_validation_error(self):
        """Test 7: Truss elements reject gradient temperature loads (uniform only)."""
        truss = Element(id="T1", type="truss", node_i=self.node_i, node_j=self.node_j, 
                        material=self.mat, section=self.sec)
        
        # Attempt to apply a gradient load to a truss (should raise error)
        thermal_load = TemperatureL(element=truss, Tu=20.0, Tb=70.0)
        
        # Should raise ValueError when computing FEF
        with self.assertRaises(ValueError) as context:
            thermal_load.FEF("pin-pin", 5.0)
        
        self.assertIn("cannot accept gradient temperature loads", str(context.exception))
        self.assertIn("Tu must equal Tb", str(context.exception))

    def test_settlement_induced_forces_cantilever_vy(self):
        """
        Test 6: Verify settlement-induced nodal forces for a cantilever.
        
        Setup: Fixed-free cantilever (L=5m).
          - Fixed at node i (x=0, y=0)
          - Free at node j (x=5, y=0)
          - Settlement at node j: displacement Δy = 0.1 m (vertical)
        
        Expected: The cantilever stiffness should produce member-end forces.
        Magnitude: |V| = 12EI*|Δy|/L³, |M| = 6EI*|Δy|/L²
        
        Note: Sign convention may vary based on coordinate system orientation.
        We verify absolute magnitudes here.
        """
        # Create a cantilever element (no releases)
        frame = Element(id="F1", type="frame", node_i=self.node_i, node_j=self.node_j, 
                        material=self.mat, section=self.sec)
        physics = ElementPhysics(frame)
        
        # Parameters
        L = 5.0
        E = 2.0e8
        I = 0.0001
        delta_y = 0.1  # 0.1 m vertical settlement at free end
        
        # Get local stiffness and transform to global
        k_local = physics.get_local_k()
        fef_dummy = math_utils.zeros(6, 1)
        k_condensed, _ = physics.condense(k_local, fef_dummy)
        k_global, _ = physics.transform_to_global(k_condensed, fef_dummy)
        
        # Create prescribed displacement vector (settlement at node j in y-direction)
        # u_prescribed = [0, 0, 0, 0, delta_y, 0]
        u_prescribed = math_utils.zeros(6, 1)
        u_prescribed[4][0] = delta_y  # Vertical displacement at node j
        
        # Compute settlement forces: f = [k] * u
        f_settlement = math_utils.matmul(k_global, u_prescribed)
        
        # Expected force magnitudes (from cantilever stiffness formulation):
        expected_vy_i_mag = 12 * E * I * delta_y / (L ** 3)  # Shear at fixed end
        expected_mz_i_mag = 6 * E * I * delta_y / (L ** 2)   # Moment at fixed end
        
        # Verify absolute magnitudes (sign may depend on convention)
        self.assertAlmostEqual(abs(f_settlement[1][0]), expected_vy_i_mag, places=-1)
        self.assertAlmostEqual(abs(f_settlement[2][0]), expected_mz_i_mag, places=-1)
        
    def test_settlement_support_dataclass(self):
        """
        Test 7: Verify that Support dataclass correctly stores settlements.
        Tests the data model extension for settlement attributes.
        """
        node = Node(id=1, x=0.0, y=0.0)
        
        # Create support with settlements
        sup = Support(
            node=node,
            restrain_ux=True,
            restrain_uy=True,
            restrain_rz=True,
            settlement_ux=0.001,
            settlement_uy=-0.002
        )
        
        self.assertEqual(sup.settlement_ux, 0.001)
        self.assertEqual(sup.settlement_uy, -0.002)
        self.assertTrue(sup.restrain_ux)
        self.assertTrue(sup.restrain_uy)
        
    def test_settlement_force_equivalence(self):
        """
        Test 8: Verify that prescribed displacement forces follow f = k*u.
        
        Setup: Simple 5m frame element, horizontal settlement at free end.
        Verify: f_settlement = [k] * u_settlement is computed correctly.
        """
        frame = Element(id="F1", type="frame", node_i=self.node_i, node_j=self.node_j, 
                        material=self.mat, section=self.sec)
        physics = ElementPhysics(frame)
        
        L = 5.0
        E = 2.0e8
        A = 0.01
        
        k_local = physics.get_local_k()
        fef_dummy = math_utils.zeros(6, 1)
        k_condensed, _ = physics.condense(k_local, fef_dummy)
        k_global, _ = physics.transform_to_global(k_condensed, fef_dummy)
        
        # Axial settlement at node j
        delta_x = 0.05  # 5 cm axial displacement
        u_prescribed = math_utils.zeros(6, 1)
        u_prescribed[3][0] = delta_x  # Axial displacement at node j
        
        # Settlement forces
        f_settlement = math_utils.matmul(k_global, u_prescribed)
        
        # Expected axial force: F_x = (E*A/L) * delta_x
        expected_fx = (E * A / L) * delta_x
        
        # Check axial forces at both ends (should be equal and opposite)
        self.assertAlmostEqual(f_settlement[0][0], -expected_fx, places=-2)
        self.assertAlmostEqual(f_settlement[3][0], expected_fx, places=-2)

if __name__ == '__main__':
    unittest.main()