def calculate_baseline_score(feasible):
    if feasible:
        return feasible[0], "First feasible idle drone in fleet order"
    return None, "No drone available with sufficient time and return energy; retry later"
