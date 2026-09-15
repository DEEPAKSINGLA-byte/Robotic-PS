import os

with open('simulation.py', 'r') as f:
    lines = f.readlines()

def get_func(func_name, start_index=0):
    start = -1
    for i in range(start_index, len(lines)):
        if lines[i].startswith(f'def {func_name}('):
            start = i
            break
    if start == -1:
        return []
    
    end = start + 1
    while end < len(lines):
        if lines[end].startswith('def ') or lines[end].startswith('class ') or (lines[end].strip() and not lines[end].startswith(' ') and not lines[end].startswith('\t') and not lines[end].startswith('#') and not lines[end].startswith('}')):
            if lines[end].startswith('FULL_CHARGE_TIME'):
                pass
            elif lines[end].startswith('DEFAULT_TIME_SCALE'):
                pass
            elif lines[end].startswith('SIMULATION_STEP'):
                pass
            else:
                break
        end += 1
        
    return lines[start:end]

# Let's just create a more robust extractor:
import ast
class FuncExtractor(ast.NodeVisitor):
    def __init__(self):
        self.nodes = {}
    def visit_FunctionDef(self, node):
        self.nodes[node.name] = (node.lineno - 1, node.end_lineno)
        self.generic_visit(node)
    def visit_Assign(self, node):
        for t in node.targets:
            if isinstance(t, ast.Name):
                self.nodes[t.id] = (node.lineno - 1, node.end_lineno)
        self.generic_visit(node)

with open('simulation.py', 'r') as f:
    source = f.read()
    
tree = ast.parse(source)
extractor = FuncExtractor()
extractor.visit(tree)

def get_source(names):
    out = []
    # get lines for each name
    for name in names:
        if name in extractor.nodes:
            start, end = extractor.nodes[name]
            out.extend(lines[start:end])
            out.append('\n')
        else:
            print(f"Warning: {name} not found")
    return "".join(out)

physics_names = [
    'move_towards',
    'calculate_battery_consumption',
    'pixels_to_meters',
    'meters_to_pixels',
    'calculate_speed',
    'calculate_delivery_time',
    'calculate_mission_time',
    'calculate_mission_distance',
    'calculate_required_battery',
    'mission_departure_energy', # wait, doesn't exist?
    'is_mission_feasible'
]

charging_names = [
    'FULL_CHARGE_TIME',
    'apply_battery_degradation',
    'calculate_charge_time',
    'get_free_charging_pad',
    'start_charging',
    'update_charging',
    'consume_battery',
    'manage_charging_infrastructure',
    'charging_candidates_at_base',
    'plan_fleet_preparation'
]

features_names = [
    'calculate_assignment_features'
]

v1_names = [
    'calculate_assignment_score'
]

v2_names = [
    'calculate_assignment_score_v2'
]

import json
out_files = {}

out_files['physics.py'] = 'import math\nfrom drone_sim import BASE\n\n' + get_source(physics_names)

out_files['charging.py'] = 'import math\nfrom drone_sim import BASE, DroneState\nfrom physics import *\n\n' + get_source(charging_names)

out_files['assignment_costs/features.py'] = 'from physics import *\n\n' + get_source(features_names)

out_files['assignment_costs/v1_cost.py'] = get_source(v1_names)

out_files['assignment_costs/v2_cost.py'] = get_source(v2_names)

out_files['assignment_costs/baseline_cost.py'] = 'def calculate_baseline_score(feasible):\n    if feasible:\n        return feasible[0], "First feasible idle drone in fleet order"\n    return None, "No drone available with sufficient time and return energy; retry later"\n'

removed_names = set(physics_names + charging_names + features_names + v1_names + v2_names)

new_sim_lines = []
skip_until = -1

for i, line in enumerate(lines):
    if i < skip_until:
        continue
    removed = False
    for name in removed_names:
        if name in extractor.nodes:
            start, end = extractor.nodes[name]
            if i == start:
                skip_until = end
                removed = True
                break
    if not removed:
        new_sim_lines.append(line)

imports = """
from physics import *
from charging import *
from assignment_costs.features import *
from assignment_costs.v1_cost import *
from assignment_costs.v2_cost import *
from assignment_costs.baseline_cost import *
"""

new_sim_lines.insert(8, imports)

out_files['simulation.py'] = "".join(new_sim_lines)

with open('/tmp/refactor_out.json', 'w') as f:
    json.dump(out_files, f)

print("Saved to /tmp/refactor_out.json")

