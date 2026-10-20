"""Nominal wheel-distance conversion; no hardware or motor commands."""
import math

def signed_delta(new, old):
    return (int(new)-int(old)+(1<<31)) % (1<<32) - (1<<31)

def geometry(config):
    values = [config['wheel_diameter_mm'], config['gear_ratio'],
              config['encoder_ppr_motor_shaft'], config['decoder_edges_per_cycle']]
    if any(not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError('Positive finite wheel/encoder parameters required')
    cpr = values[1]*values[2]*values[3]
    circumference = math.pi*values[0]
    return {'counts_per_wheel_revolution': cpr, 'wheel_circumference_mm': circumference,
            'mm_per_count': circumference/cpr}

def displacement(config, start, end):
    g = geometry(config)
    output = {'basis': config['parameter_basis'], 'calibrated': False, **g}
    for wheel in ('left', 'right'):
        channel = config[wheel+'_channel']
        name = 'encoder_'+channel.lower()
        ticks = signed_delta(end[name], start[name])*config['forward_raw_sign'][channel]
        output[wheel+'_forward_counts'] = ticks
        output[wheel+'_distance_mm'] = ticks*g['mm_per_count']
    output['mean_wheel_distance_mm'] = (output['left_distance_mm']+output['right_distance_mm'])/2
    output['elapsed_ms'] = (end['ms']-start['ms']) % (1<<32)
    return output
