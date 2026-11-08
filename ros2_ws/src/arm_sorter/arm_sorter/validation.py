import math

def validate_points(names, points, config):
    if not names or len(set(names)) != len(names) or not set(names) <= set(config['joint_names']):
        raise ValueError('Unknown or duplicate joints')
    if not points: raise ValueError('Empty trajectory')
    previous=-1.0
    for seconds, positions in points:
        if not math.isfinite(seconds) or seconds < 0 or seconds <= previous:
            raise ValueError('Trajectory times must increase')
        if len(positions)!=len(names): raise ValueError('Position count mismatch')
        for name, q in zip(names, positions):
            lo, hi=config['limits'][name]
            if not math.isfinite(q) or not lo <= q <= hi: raise ValueError(f'Joint limit: {name}')
        previous=seconds
