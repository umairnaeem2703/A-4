# src/dof_optimizer.py

from parser import StructuralModel
from banded_solver import UnstableStructureError

class DOFOptimizer:
    def __init__(self, model: StructuralModel):
        self.model = model
        self.num_equations = 0
        self.semi_bandwidth = 0
        self.full_bandwidth = 0

    def optimize(self) -> tuple[int, int, int]:
        """Runs the optimization and returns (num_equations, semi_bandwidth, full_bandwidth)."""
        self._validate_structure()
        adj_list = self._build_adjacency_list()
        rcm_order = self._reverse_cuthill_mckee(adj_list)
        self._assign_dofs(rcm_order)
        self._calculate_bandwidth()
        return self.num_equations, self.semi_bandwidth, self.full_bandwidth

    def _validate_structure(self):
        """Per-solve topology checks. Identifies unsolvable structural configurations."""
        # Fatal Check 1: Zero Supports
        if not self.model.supports:
            raise UnstableStructureError(
                "No boundary conditions defined. "
                "The structure is entirely unsupported — "
                "rigid body modes exist. Global stiffness matrix K will be singular."
            )

        fatal_errors = []

        # Fatal Check 2: Global X-Restraint
        if not any(s.restrain_ux for s in self.model.supports.values()):
            fatal_errors.append(
                "No support restrains global x-translation. "
                "The structure can sway freely in the x-direction. "
                "Ensure at least one node is pinned or fixed."
            )

        # Connectivity Analysis
        adj = {}
        for el in self.model.elements.values():
            ni, nj = el.node_i.id, el.node_j.id
            adj.setdefault(ni, set()).add(nj)
            adj.setdefault(nj, set()).add(ni)

        # BFS to find connected components
        unvisited = set(adj)
        components = []
        while unvisited:
            start = next(iter(unvisited))
            queue, component = [start], set()
            while queue:
                node = queue.pop(0)
                if node in component:
                    continue
                component.add(node)
                queue.extend(n for n in adj.get(node, []) if n not in component)
            unvisited -= component
            components.append(frozenset(component))

        # Fatal Check 3: Floating Sub-structures
        for component in components:
            has_support = any(n in self.model.supports for n in component)
            if not has_support:
                floating_members = sorted(
                    el.id for el in self.model.elements.values()
                    if el.node_i.id in component or el.node_j.id in component
                )
                fatal_errors.append(
                    f"Member(s) [{', '.join(floating_members)}] form a floating "
                    "sub-structure with no supports. Their DOFs will cause singular rows in K."
                )

        if fatal_errors:
            raise UnstableStructureError(
                "\n".join(f"  ↳ {e}" for e in fatal_errors)
            )

        if len(components) > 1:
            print(f"INFO: {len(components)} disconnected sub-structures detected. "
                  "All parts are independently supported and solvable.")

    def _has_rotational_stiffness(self, node_id: int) -> bool:
        """
        Checks if a node receives rotational stiffness from any connected member.
        This prevents 'spinning node' mechanisms (singular matrix) when multiple 
        hinged members meet at a free node.
        """
        for el in self.model.elements.values():
            if el.type == 'frame':
                if el.node_i.id == node_id and not el.release_start:
                    return True
                if el.node_j.id == node_id and not el.release_end:
                    return True
        return False

    def _build_adjacency_list(self) -> dict:
        """Creates an adjacency graph of node connectivity, excluding fully restrained nodes."""
        adj = {}
        
        for n_id, node in self.model.nodes.items():
            support = self.model.supports.get(n_id)
            # Check if node has at least one active DOF
            has_active = not (support and support.restrain_ux and support.restrain_uy and support.restrain_rz)
            
            if has_active or any(el.node_i.id == n_id or el.node_j.id == n_id for el in self.model.elements.values()):
                adj[n_id] = set()
        
        for el in self.model.elements.values():
            if el.node_i.id in adj and el.node_j.id in adj:
                adj[el.node_i.id].add(el.node_j.id)
                adj[el.node_j.id].add(el.node_i.id)
                
        return {k: list(v) for k, v in adj.items()}

    def _reverse_cuthill_mckee(self, adj: dict) -> list:
        """Orders nodes using RCM with coordinate tie-breaking."""
        if not adj:
            return []
            
        degrees = {node: len(neighbors) for node, neighbors in adj.items()}
        
        def sort_key(n_id):
            node = self.model.nodes[n_id]
            return (degrees[n_id], float(node.x), float(node.y), int(n_id))

        unvisited = set(adj.keys())
        result = []

        while unvisited:
            start_node = min(unvisited, key=sort_key)
            queue = [start_node]
            unvisited.remove(start_node)

            while queue:
                current = queue.pop(0)
                result.append(current)

                neighbors = [n for n in adj[current] if n in unvisited]
                neighbors.sort(key=sort_key)
                
                for neighbor in neighbors:
                    queue.append(neighbor)
                    unvisited.remove(neighbor)
                    
        return result[::-1]

    def _assign_dofs(self, rcm_order: list):
        """Assigns equation numbers to active DOFs based on boundary conditions."""
        self.num_equations = 0
        
        for node in self.model.nodes.values():
            node.dofs = [-1, -1, -1]
            
        for node_id in rcm_order:
            node = self.model.nodes[node_id]
            support = self.model.supports.get(node_id)
            
            if not (support and support.restrain_ux):
                node.dofs[0] = self.num_equations
                self.num_equations += 1
                
            if not (support and support.restrain_uy):
                node.dofs[1] = self.num_equations
                self.num_equations += 1
                
            # Auto-constrain orphan rotational DOFs by ignoring them
            if self._has_rotational_stiffness(node_id) and not (support and support.restrain_rz):
                node.dofs[2] = self.num_equations
                self.num_equations += 1

    def _calculate_bandwidth(self):
        """Calculates semi-bandwidth m = max(|DOF_a - DOF_b|) + 1, and full bandwidth."""
        max_m = 0
        for el in self.model.elements.values():
            active_dofs = [dof for dof in (el.node_i.dofs + el.node_j.dofs) if dof >= 0]
            
            if active_dofs:
                m = max(active_dofs) - min(active_dofs) + 1
                if m > max_m:
                    max_m = m
                    
        self.semi_bandwidth = max_m if max_m > 0 else 1
        self.full_bandwidth = 2 * self.semi_bandwidth - 1