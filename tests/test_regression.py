# tests/test_regression.py

import sys
import os
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from parser import XMLParser, StructuralModel, Node, Element, Material, Section, Support, LoadCase, NodalLoad
from dof_optimizer import DOFOptimizer
from matrix_assembly import MatrixAssembler
from banded_solver import BandedSolver
from post_processor import PostProcessor

class TestRegression(unittest.TestCase):

    def setUp(self):
        self.disp_tol = 1e-3   # Standard Tolerance for SAP2000 Displacements
        self.force_tol = 0.5   # Standard Tolerance for SAP2000 Forces
        self.exact_tol = 1e-5  # Stricter Tolerance for Analytical/Hardcoded checks

    def _parse_sap2000(self, xml_path):
        """Helper method to dynamically parse SAP2000 results from a text file."""
        disp = {}
        react = {}
        forces = {}
        current_table = None
        
        with open(xml_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith("Table:  Joint Displacements"):
                    current_table = "disp"
                    continue
                if line.startswith("Table:  Joint Reactions"):
                    current_table = "react"
                    continue
                if line.startswith("Table:  Element Forces - Frames"):
                    current_table = "forces"
                    continue
                if not line or line.startswith("SAP2000") or line.startswith("Table:"):
                    continue
                    
                tokens = line.split()
                if len(tokens) < 9:
                    continue
                    
                try:
                    first_val = float(tokens[0])
                except ValueError:
                    continue
                    
                if current_table == "disp":
                    node_id = int(tokens[0])
                    # SAP2000 (XZ Plane): U1=X, U3=Y, R2=Rot_Y (Opposite to our RZ)
                    disp[node_id] = (float(tokens[3]), float(tokens[5]), -float(tokens[7]))
                    
                elif current_table == "react":
                    node_id = int(tokens[0])
                    # SAP2000 (XZ Plane): F1=Fx, F3=Fy, M2=Mz (Opposite to our Mz)
                    react[node_id] = (float(tokens[3]), float(tokens[5]), -float(tokens[7]))
                    
                elif current_table == "forces" and len(tokens) >= 10:
                    frame_id = tokens[0]
                    station = float(tokens[1])
                    
                    P = float(tokens[4])
                    V2 = float(tokens[5])
                    M3 = float(tokens[9])
                    
                    if frame_id not in forces:
                        forces[frame_id] = {'min_st': station, 'max_st': station, 'i': (P, V2, M3), 'j': (P, V2, M3)}
                    else:
                        if station < forces[frame_id]['min_st']:
                            forces[frame_id]['min_st'] = station
                            forces[frame_id]['i'] = (P, V2, M3)
                        if station > forces[frame_id]['max_st']:
                            forces[frame_id]['max_st'] = station
                            forces[frame_id]['j'] = (P, V2, M3)
                            
        return disp, react, forces

    def run_full_analysis(self, model, load_case="LC1"):
        """Reusable pipeline"""
        opt = DOFOptimizer(model)
        num_eq, semi_bw, _ = opt.optimize()

        assembler = MatrixAssembler(model, num_eq, semi_bw)
        K_banded, F_global = assembler.assemble(load_case)

        solver = BandedSolver(K_banded, F_global, semi_bw)
        D_active = solver.solve()

        processor = PostProcessor(model, D_active, load_case)

        return processor, F_global

    # ==========================================================
    # TEST 1 — TRUSS (SAP2000 Benchmark)
    # ==========================================================
    def test_regression_01_truss_displacements_and_reactions(self):
        xml_path = os.path.join(os.path.dirname(__file__), "../data/example1_case1_truss.xml")
        sap_path = os.path.join(os.path.dirname(__file__), "../data/example1_case1_truss_sap2000.txt")
        if not os.path.exists(xml_path) or not os.path.exists(sap_path):
            self.skipTest("Missing input or SAP2000 text file")

        sap_disp, sap_react, sap_forces = self._parse_sap2000(sap_path)
        model = XMLParser(xml_path).parse()
        processor, _ = self.run_full_analysis(model)

        disp = processor.displacements
        reactions = processor.reactions
        local_forces = processor.member_forces

        self.assertAlmostEqual(disp[1][0], sap_disp[1][0], delta=self.disp_tol)
        self.assertAlmostEqual(disp[1][1], sap_disp[1][1], delta=self.disp_tol)
        self.assertAlmostEqual(abs(reactions[1][0]), abs(sap_react[1][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(reactions[1][1]), abs(sap_react[1][1]), delta=self.force_tol)
        self.assertIn("T1", local_forces)
        
        f_T1 = local_forces["T1"]
        sap_f = sap_forces["1"]
        self.assertAlmostEqual(abs(f_T1[0][0]), abs(sap_f['i'][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_T1[2][0]), abs(sap_f['j'][0]), delta=self.force_tol)

    # ==========================================================
    # TEST 2 — FRAME (SAP2000 Benchmark)
    # ==========================================================
    def test_regression_02_frame_full_validation(self):
        xml_path = os.path.join(os.path.dirname(__file__), "../data/example2_frame.xml")
        sap_path = os.path.join(os.path.dirname(__file__), "../data/example2_frame_sap2000.txt")
        if not os.path.exists(xml_path) or not os.path.exists(sap_path):
            self.skipTest("Missing input or SAP2000 text file")

        sap_disp, sap_react, sap_forces = self._parse_sap2000(sap_path)
        model = XMLParser(xml_path).parse()
        processor, _ = self.run_full_analysis(model)

        disp = processor.displacements
        reactions = processor.reactions
        local_forces = processor.member_forces

        self.assertAlmostEqual(disp[1][0], sap_disp[1][0], delta=self.disp_tol)
        self.assertAlmostEqual(disp[1][1], sap_disp[1][1], delta=self.disp_tol)
        self.assertAlmostEqual(disp[1][2], sap_disp[1][2], delta=self.disp_tol)
        self.assertAlmostEqual(abs(reactions[1][0]), abs(sap_react[1][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(reactions[1][1]), abs(sap_react[1][1]), delta=self.force_tol)
        self.assertAlmostEqual(abs(reactions[1][2]), abs(sap_react[1][2]), delta=self.force_tol)

        self.assertIn("F1", local_forces)
        f_F1 = local_forces["F1"]
        sap_f = sap_forces["1"]
        self.assertAlmostEqual(abs(f_F1[0][0]), abs(sap_f['i'][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_F1[3][0]), abs(sap_f['j'][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_F1[1][0]), abs(sap_f['i'][1]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_F1[4][0]), abs(sap_f['j'][1]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_F1[2][0]), abs(sap_f['i'][2]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_F1[5][0]), abs(sap_f['j'][2]), delta=self.force_tol)

    # ==========================================================
    # TEST 3 — MIXED FRAME-TRUSS (SAP2000 Benchmark)
    # ==========================================================
    def test_regression_03_mixed_structure_results(self):
        xml_path = os.path.join(os.path.dirname(__file__), "../data/example3_frame_truss.xml")
        sap_path = os.path.join(os.path.dirname(__file__), "../data/example3_frame_truss_sap2000.txt")
        if not os.path.exists(xml_path) or not os.path.exists(sap_path):
            self.skipTest("Missing input or SAP2000 text file")

        sap_disp, sap_react, sap_forces = self._parse_sap2000(sap_path)
        model = XMLParser(xml_path).parse()
        processor, _ = self.run_full_analysis(model)

        disp = processor.displacements
        reactions = processor.reactions
        local_forces = processor.member_forces

        self.assertAlmostEqual(disp[1][0], sap_disp[1][0], delta=self.disp_tol)
        self.assertAlmostEqual(disp[1][1], sap_disp[1][1], delta=self.disp_tol)
        self.assertAlmostEqual(disp[1][2], sap_disp[1][2], delta=self.disp_tol)
        self.assertAlmostEqual(abs(reactions[1][0]), abs(sap_react[1][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(reactions[1][1]), abs(sap_react[1][1]), delta=self.force_tol)
        self.assertAlmostEqual(abs(reactions[1][2]), abs(sap_react[1][2]), delta=self.force_tol)

        self.assertIn("F1", local_forces)
        f_F1 = local_forces["F1"]
        sap_f = sap_forces["1"]
        self.assertAlmostEqual(abs(f_F1[0][0]), abs(sap_f['i'][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_F1[3][0]), abs(sap_f['j'][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_F1[1][0]), abs(sap_f['i'][1]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_F1[4][0]), abs(sap_f['j'][1]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_F1[2][0]), abs(sap_f['i'][2]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_F1[5][0]), abs(sap_f['j'][2]), delta=self.force_tol)

    # ==========================================================
    # TEST 4 — EG1 TRUSS TEMPERATURE (Hardcoded Answers)
    # ==========================================================
    def test_regression_04_eg1_truss_temp(self):
        """Validates the 3-truss temperature problem against exact whiteboard answers."""
        xml_path = os.path.join(os.path.dirname(__file__), "../data/eg1_truss_temp.xml")
        if not os.path.exists(xml_path):
            self.skipTest("Missing input file eg1_truss_temp.xml")

        model = XMLParser(xml_path).parse()
        processor, F_global = self.run_full_analysis(model)

        # Expected Vector F (Equivalent Nodal Loads derived from FEF)
        # N1 is free in X and Y.
        n1 = model.nodes[1]
        self.assertAlmostEqual(F_global[n1.dofs[0]][0], -72.08, delta=0.01)
        self.assertAlmostEqual(F_global[n1.dofs[1]][0], -32.23, delta=0.01)

        # Expected Displacements u (in metres)
        disp = processor.displacements
        self.assertAlmostEqual(disp[1][0], -0.000405, delta=self.exact_tol)
        self.assertAlmostEqual(disp[1][1], -0.000070, delta=self.exact_tol)

    # ==========================================================
    # TEST 5 — EG2 BEAM TEMPERATURE (Hardcoded Answers)
    # ==========================================================
    def test_regression_05_eg2_beam_temp(self):
        """Validates the 2-span beam thermal gradient problem against exact whiteboard answers."""
        xml_path = os.path.join(os.path.dirname(__file__), "../data/eg2_beam_temp.xml")
        if not os.path.exists(xml_path):
            self.skipTest("Missing input file eg2_beam_temp.xml")

        model = XMLParser(xml_path).parse()
        processor, F_global = self.run_full_analysis(model)

        # Expected Vector F (Equivalent Moments at nodes 1 and 2)
        n1 = model.nodes[1]
        n2 = model.nodes[2]
        self.assertAlmostEqual(F_global[n1.dofs[2]][0], 12.0, delta=0.01)
        self.assertAlmostEqual(F_global[n2.dofs[2]][0], -6.0, delta=0.01)

        # Expected Displacements u (Rotations in radians)
        disp = processor.displacements
        self.assertAlmostEqual(disp[1][2], 0.00086, delta=self.exact_tol)
        self.assertAlmostEqual(disp[2][2], -0.00052, delta=self.exact_tol)

    # ==========================================================
    # TEST 6 — Q2A SUPPORT SETTLEMENT (SAP2000 Benchmark)
    # ==========================================================
    def test_regression_06_q2a_support_settlement(self):
        xml_path = os.path.join(os.path.dirname(__file__), "../data/Assignment_4_Q2a.xml")
        sap_path = os.path.join(os.path.dirname(__file__), "../data/q2_a_sap2000.txt")
        if not os.path.exists(xml_path) or not os.path.exists(sap_path):
            self.skipTest("Missing input or SAP2000 text file")

        sap_disp, sap_react, sap_forces = self._parse_sap2000(sap_path)
        model = XMLParser(xml_path).parse()
        processor, _ = self.run_full_analysis(model)

        disp = processor.displacements
        reactions = processor.reactions
        local_forces = processor.member_forces

        self.assertAlmostEqual(disp[2][0], sap_disp[2][0], delta=self.disp_tol)
        self.assertAlmostEqual(disp[2][1], sap_disp[2][1], delta=self.disp_tol)
        self.assertAlmostEqual(disp[2][2], sap_disp[2][2], delta=self.disp_tol)
        self.assertAlmostEqual(disp[3][1], sap_disp[3][1], delta=self.disp_tol)
        self.assertAlmostEqual(disp[5][1], sap_disp[5][1], delta=self.disp_tol)

        self.assertAlmostEqual(abs(reactions[1][0]), abs(sap_react[1][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(reactions[1][1]), abs(sap_react[1][1]), delta=self.force_tol)
        self.assertAlmostEqual(abs(reactions[5][0]), abs(sap_react[5][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(reactions[5][1]), abs(sap_react[5][1]), delta=self.force_tol)
        self.assertAlmostEqual(abs(reactions[6][0]), abs(sap_react[6][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(reactions[6][1]), abs(sap_react[6][1]), delta=self.force_tol)

        self.assertIn("AB", local_forces)
        f_AB = local_forces["AB"]
        sap_f1 = sap_forces["1"]
        self.assertAlmostEqual(abs(f_AB[0][0]), abs(sap_f1['i'][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_AB[3][0]), abs(sap_f1['j'][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_AB[1][0]), abs(sap_f1['i'][1]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_AB[4][0]), abs(sap_f1['j'][1]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_AB[2][0]), abs(sap_f1['i'][2]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_AB[5][0]), abs(sap_f1['j'][2]), delta=self.force_tol)

        self.assertIn("FC", local_forces)
        f_FC = local_forces["FC"]
        sap_f5 = sap_forces["5"]
        self.assertAlmostEqual(abs(f_FC[0][0]), abs(sap_f5['i'][0]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_FC[2][0]), abs(sap_f5['j'][0]), delta=self.force_tol)

    # ==========================================================
    # TEST 7 — Q2B THERMAL LOADING (SAP2000 Benchmark)
    # ==========================================================
    def test_regression_07_q2b_thermal_loading(self):
        xml_path = os.path.join(os.path.dirname(__file__), "../data/Assignment_4_Q2b.xml")
        sap_path = os.path.join(os.path.dirname(__file__), "../data/q2_b_sap2000.txt")
        if not os.path.exists(xml_path) or not os.path.exists(sap_path):
            self.skipTest("Missing input or SAP2000 text file")

        sap_disp, sap_react, sap_forces = self._parse_sap2000(sap_path)
        model = XMLParser(xml_path).parse()
        processor, _ = self.run_full_analysis(model)

        disp = processor.displacements
        reactions = processor.reactions
        local_forces = processor.member_forces

        q2b_force_tol = 4.0

        self.assertAlmostEqual(disp[2][2], sap_disp[2][2], delta=self.disp_tol)
        self.assertAlmostEqual(disp[3][2], sap_disp[3][2], delta=self.disp_tol)

        self.assertAlmostEqual(abs(reactions[1][0]), abs(sap_react[1][0]), delta=q2b_force_tol)
        self.assertAlmostEqual(abs(reactions[3][0]), abs(sap_react[3][0]), delta=q2b_force_tol)
        self.assertAlmostEqual(abs(reactions[3][1]), abs(sap_react[3][1]), delta=self.force_tol)

        self.assertIn("ABC", local_forces)
        f_ABC = local_forces["ABC"]
        sap_f1 = sap_forces["1"]
        self.assertAlmostEqual(abs(f_ABC[0][0]), abs(sap_f1['i'][0]), delta=q2b_force_tol)
        self.assertAlmostEqual(abs(f_ABC[3][0]), abs(sap_f1['j'][0]), delta=q2b_force_tol)
        self.assertAlmostEqual(abs(f_ABC[4][0]), abs(sap_f1['j'][1]), delta=self.force_tol)
        self.assertAlmostEqual(abs(f_ABC[5][0]), abs(sap_f1['j'][2]), delta=self.force_tol)

if __name__ == '__main__':
    unittest.main()