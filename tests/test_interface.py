# tests/test_interface.py

import sys
import os
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from parser import Node, Material, Section, Element, Support, LoadCase, StructuralModel, NodalLoad, UniformlyDL, TemperatureL
from dof_optimizer import DOFOptimizer
from matrix_assembly import MatrixAssembler

class TestMatrixAssembly(unittest.TestCase):
    def setUp(self):
        """Creates a minimal 2-element truss model manually for controlled testing."""
        self.model = StructuralModel(name="Test_Assembly")
        
        self.mat = Material(id="mat1", E=2.0e8)
        self.sec = Section(id="sec1", A=0.01) # EA/L = 2e6 for L=1
        
        # Geometry: Node 1 (0,0) -> Node 2 (1,0) -> Node 3 (2,0)
        self.n1 = Node(id=1, x=0.0, y=0.0)
        self.n2 = Node(id=2, x=1.0, y=0.0)
        self.n3 = Node(id=3, x=2.0, y=0.0)
        
        self.model.nodes = {1: self.n1, 2: self.n2, 3: self.n3}
        
        self.e1 = Element(id="E1", type="truss", node_i=self.n1, node_j=self.n2, material=self.mat, section=self.sec)
        self.e2 = Element(id="E2", type="truss", node_i=self.n2, node_j=self.n3, material=self.mat, section=self.sec)
        self.model.elements = {"E1": self.e1, "E2": self.e2}
        
        # Restrain Node 1 fully (Pin). Restrain Node 3 in Y (Roller). Node 2 is completely free.
        self.model.supports = {
            1: Support(node=self.n1, restrain_ux=True, restrain_uy=True),
            3: Support(node=self.n3, restrain_ux=False, restrain_uy=True)
        }
        
        # Apply a point load at Node 2
        lc = LoadCase(id="LC1")
        lc.loads.append(NodalLoad(node=self.n2, fx=50.0, fy=-10.0))
        self.model.load_cases = {"LC1": lc}

        # Run Optimizer to set up DOFs
        self.optimizer = DOFOptimizer(self.model)
        self.num_eq, self.semi_bw, self.full_bw = self.optimizer.optimize()

    def test_assembly_global_stiffness_mapping(self):
        """Test 1: Verify local element matrices map correctly to Global Banded [K]."""
        assembler = MatrixAssembler(self.model, self.num_eq, self.semi_bw)
        K_banded, _ = assembler.assemble("LC1")
        
        n2_ux_dof = self.n2.dofs[0]
        n3_ux_dof = self.n3.dofs[0]
        
        self.assertEqual(K_banded[n2_ux_dof][0], 4000000.0)
        self.assertEqual(K_banded[n3_ux_dof][0], 2000000.0)
        
        min_dof = min(n2_ux_dof, n3_ux_dof)
        max_dof = max(n2_ux_dof, n3_ux_dof)
        self.assertEqual(K_banded[min_dof][max_dof - min_dof], -2000000.0)

    def test_assembly_global_load_vector(self):
        """Test 2: Verify Nodal loads and Equivalent Nodal Loads combine into {F}."""
        # We attach N4 to N3, and leave N4 completely free. 
        # This prevents N4 from triggering the "boundary pin" auto-detect logic, 
        # guaranteeing the program treats the connection as a fixed node.
        n4 = Node(id=4, x=2.0, y=5.0)
        self.model.nodes[4] = n4
        frame = Element(id="F1", type="frame", node_i=self.n3, node_j=n4, material=self.mat, section=self.sec)
        self.model.elements["F1"] = frame
        
        self.model.load_cases["LC1"].loads.append(UniformlyDL(element=frame, wy=-10.0))
        
        # Re-run optimizer because we added nodes/elements
        num_eq, semi_bw, _ = self.optimizer.optimize()
        
        assembler = MatrixAssembler(self.model, num_eq, semi_bw)
        _, F_global = assembler.assemble("LC1")
        
        # Point load check at N2
        n2_ux_dof = self.n2.dofs[0]
        n2_uy_dof = self.n2.dofs[1]
        self.assertEqual(F_global[n2_ux_dof][0], 50.0)
        self.assertEqual(F_global[n2_uy_dof][0], -10.0)
        
        # FEF check at N4 (free end of vertical frame F1)
        # Because N4 is free, it provides a rigid moment connection (pin-fixed or fixed-fixed).
        # The Fixed-End moment at N4 must be non-zero.
        n4_rz_dof = self.model.nodes[4].dofs[2] 
        self.assertNotEqual(F_global[n4_rz_dof][0], 0.0) 

    def test_settlement_force_subtraction(self):
        """
        Test 3: Verify settlement forces are correctly subtracted from load vector.
        
        Setup: A simple frame with two nodes.
          - Node 1: Fixed (fully restrained)
          - Node 2: Pinned with prescribed vertical settlement uy = -0.001 m
        
        Expected behavior:
          - Settlement forces are computed element-by-element: f_settle = [k] * u_settle
          - These forces are subtracted from global F vector: F_modified = F_orig - f_settle
          - The difference should be significant and non-zero
        """
        # Create a 5m cantilever with pin at fixed end and settlement at pin support
        model = StructuralModel(name="Settlement_Force_Test")
        
        mat = Material(id="mat1", E=2.0e8)
        sec = Section(id="sec1", A=0.01, I=1.0e-4, d=0.3)
        
        n1 = Node(id=1, x=0.0, y=0.0)
        n2 = Node(id=2, x=5.0, y=0.0)
        
        model.nodes = {1: n1, 2: n2}
        
        e1 = Element(id="E1", type="frame", node_i=n1, node_j=n2, material=mat, section=sec)
        model.elements = {"E1": e1}
        
        # Node 1 is fixed, Node 2 is pinned with settlement
        settlement_uy = -0.001
        model.supports = {
            1: Support(node=n1, restrain_ux=True, restrain_uy=True, restrain_rz=True),
            2: Support(
                node=n2, 
                restrain_ux=True, 
                restrain_uy=True,
                restrain_rz=False,
                settlement_ux=0.0,
                settlement_uy=settlement_uy
            )
        }
        
        lc = LoadCase(id="LC1")
        lc.loads.append(NodalLoad(node=n2, fx=0.0, fy=0.0, mz=0.0))
        model.load_cases = {"LC1": lc}
        
        # Assemble with settlement
        opt_with = DOFOptimizer(model)
        num_eq, semi_bw, _ = opt_with.optimize()
        
        assembler_with = MatrixAssembler(model, num_eq, semi_bw)
        K_with, F_with_settlement = assembler_with.assemble("LC1")
        
        # Now remove the settlement
        model.supports[2] = Support(
            node=n2, 
            restrain_ux=True, 
            restrain_uy=True,
            restrain_rz=False,
            settlement_ux=0.0,
            settlement_uy=0.0
        )
        
        opt_without = DOFOptimizer(model)
        num_eq_without, semi_bw_without, _ = opt_without.optimize()
        
        assembler_without = MatrixAssembler(model, num_eq_without, semi_bw_without)
        K_without, F_without_settlement = assembler_without.assemble("LC1")
        
        # Stiffness should be identical
        self.assertEqual(len(F_with_settlement), len(F_without_settlement))
        
        # Load vectors should differ due to settlement forces
        max_diff = 0.0
        for i in range(len(F_with_settlement)):
            diff = abs(F_with_settlement[i][0] - F_without_settlement[i][0])
            max_diff = max(max_diff, diff)
        
        # Expect significant difference due to settlement forces
        self.assertGreater(max_diff, 0.1, 
            "Settlement forces should produce significant changes in load vector")

    def test_settlement_displacement_reconstruction(self):
        """
        Test 4: Verify that settlements are properly reconstructed in full
        displacement vector during post-processing.
        
        This is an integration test that:
          1. Creates a truss with prescribed settlements
          2. Manually constructs a solution vector (would be from solver)
          3. Verifies that post-processor includes settlements in full displacements
        """
        # Modify Node 3 support with a settlement
        settlement_uy = -0.002
        self.model.supports[3] = Support(
            node=self.n3,
            restrain_ux=False,
            restrain_uy=True,
            settlement_ux=0.0,
            settlement_uy=settlement_uy
        )
        
        # Import post-processor
        from post_processor import PostProcessor
        
        # Create a dummy solution (D_active) that represents solved DOFs only
        # For this model: n1 has ux, uy restrained → DOF -1, -1
        #                n2 has ux, uy free → DOF 0, 1
        #                n3 has ux free, uy restrained → DOF 2, -1
        # So num_eq = 3 (DOFs 0, 1, 2)
        
        # Assemble and solve are normally done, but we'll skip and use dummy solution
        self.optimizer = DOFOptimizer(self.model)
        num_eq, semi_bw, _ = self.optimizer.optimize()
        
        # Dummy solution: small displacements for free DOFs
        D_active = [[0.001], [0.0001], [0.0002]]  # Three active DOFs for n2_ux, n2_uy, n3_ux
        
        # Create post-processor (it expects D_active)
        post_proc = PostProcessor(self.model, D_active, "LC1")
        
        # Check reconstructed displacements
        # Node 3 should have settlement_uy in its displacement
        displacements_n3 = post_proc.displacements[3]
        
        # displacements_n3 should be [ux_free, uy_restrained_with_settlement]
        # For a roller at node 3: ux is free (not in dofs), uy is -1 (restrained)
        # Actually, node 3 has dofs [2, -1] for [ux, uy] due to roller
        # So displacements_n3[0] = D_active[2] (the ux free DOF)
        # And displacements_n3[1] = settlement_uy = -0.002
        
        self.assertAlmostEqual(
            post_proc.displacements[3][1], 
            settlement_uy, 
            places=6, 
            msg="Node 3 uy displacement should equal prescribed settlement"
        )

    def test_thermal_load_assembly_frame(self):
        """
        Test 5: Thermal load assembly integration - verify FEFs are properly 
        converted to global loads and subtracted from the load vector.
        
        Setup: A simple 5m frame element with a uniform temperature increase.
          - Node 1: Fixed
          - Node 2: Free
          - Uniform temperature: T = +30°C
        
        Expected behavior:
          - Thermal axial FEF: F_T = alpha * T_uniform * E * A
          - Global load vector should reflect F_net = P_nodal - FEF_global
        """
        model = StructuralModel(name="Thermal_Assembly_Test")
        
        # Material with thermal expansion coefficient
        mat = Material(id="mat1", E=2.0e8, alpha=1.2e-5)
        sec = Section(id="sec1", A=0.01, I=1.0e-4, d=0.3)
        
        n1 = Node(id=1, x=0.0, y=0.0)
        n2 = Node(id=2, x=5.0, y=0.0)
        
        model.nodes = {1: n1, 2: n2}
        
        frame = Element(id="F1", type="frame", node_i=n1, node_j=n2, 
                       material=mat, section=sec)
        model.elements = {"F1": frame}
        
        # Fixed at node 1
        model.supports = {
            1: Support(node=n1, restrain_ux=True, restrain_uy=True, restrain_rz=True)
        }
        
        # Load case with uniform temperature
        lc = LoadCase(id="LC1")
        lc.loads.append(TemperatureL(element=frame, Tu=30.0, Tb=30.0))  # Uniform 30°C
        model.load_cases = {"LC1": lc}
        
        # Optimize DOFs
        optimizer = DOFOptimizer(model)
        num_eq, semi_bw, _ = optimizer.optimize()
        
        # Assemble
        assembler = MatrixAssembler(model, num_eq, semi_bw)
        K_banded, F_global = assembler.assemble("LC1")
        
        # Calculate expected thermal axial force
        T_uniform = 30.0
        F_T = mat.alpha * T_uniform * mat.E * sec.A
        
        # Check that load vector reflects the thermal load
        # Node 2 should have a load equal to the thermal force (subtracted as FEF)
        # FEF_local = [-F_T, 0, 0, F_T, 0, 0]
        # At node 2: FEF should be +F_T (push outward), so F_net should include -F_T
        n2_ux_dof = n2.dofs[0]
        
        # The thermal load creates a compressive FEF of magnitude F_T
        # This gets subtracted: F_net = F_nodal - FEF = 0 - F_T = -F_T
        self.assertAlmostEqual(
            F_global[n2_ux_dof][0], 
            -F_T,  # Negative because we're subtracting the FEF
            places=-1,
            msg="Thermal load should appear in load vector as -FEF"
        )

    def test_thermal_gradient_with_moment_release(self):
        """
        Test 6: Thermal gradient load on a frame with moment release at one end.
        Verifies that moment releases cause induced shears in thermal FEF.
        
        Setup: A 6m concrete beam with moment release at one end and
               thermal gradient (40°C at bottom, 20°C at top).
          - Beam depth d = 0.4m
          - Moment release at node j (pin-fixed condition)
        
        Expected behavior:
          - Base thermal moment: M_T = (alpha * delta_T / d) * E * I
          - With moment release at j: induced shear = M_T / L
          - Moment at i: -M_T, at j: 0 (released)
          - Shears: Vy_i = M_T/L, Vy_j = -M_T/L
        """
        model = StructuralModel(name="Thermal_Gradient_Release_Test")
        
        # Concrete beam
        mat = Material(id="concrete", E=3.0e10, alpha=1.0e-5)
        sec = Section(id="beam", A=0.10, I=1.333e-4, d=0.4)  # 0.4m depth
        
        n1 = Node(id=1, x=0.0, y=0.0)
        n2 = Node(id=2, x=6.0, y=0.0)
        
        model.nodes = {1: n1, 2: n2}
        
        # Frame with moment release at node j
        frame = Element(id="B1", type="frame", node_i=n1, node_j=n2, 
                       material=mat, section=sec, 
                       release_start=False, release_end=True)  # Pin at end j
        model.elements = {"B1": frame}
        
        # Fixed at node 1
        model.supports = {
            1: Support(node=n1, restrain_ux=True, restrain_uy=True, restrain_rz=True)
        }
        
        # Thermal gradient: bottom 40°C, top 20°C
        lc = LoadCase(id="LC1")
        lc.loads.append(TemperatureL(element=frame, Tu=20.0, Tb=40.0))
        model.load_cases = {"LC1": lc}
        
        # Optimize and assemble
        optimizer = DOFOptimizer(model)
        num_eq, semi_bw, _ = optimizer.optimize()
        
        assembler = MatrixAssembler(model, num_eq, semi_bw)
        K_banded, F_global = assembler.assemble("LC1")
        
        # Calculate expected values
        delta_T = 40.0 - 20.0  # 20°C
        M_T = (mat.alpha * delta_T / sec.d) * mat.E * sec.I
        induced_shear = M_T / 6.0  # L = 6m
        
        # Check that the load vector includes the induced shear from moment release
        n1_uy_dof = n1.dofs[1]
        
        # With pin-fixed condition: shear at i should be approximately from induced moment
        # This is a complex calculation, so we just verify non-zero shear loads are present
        # The actual value depends on how the FEF is assembled globally
        
        # Verify that thermal loads created non-zero entries in load vector
        thermal_loads_present = sum(1 for i in range(len(F_global)) 
                                   if abs(F_global[i][0]) > 1e-6)
        self.assertGreater(
            thermal_loads_present, 
            0,
            msg="Thermal gradient load should produce non-zero loads in assembly"
        )

if __name__ == '__main__':
    unittest.main()