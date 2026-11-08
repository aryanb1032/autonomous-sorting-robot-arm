#!/usr/bin/env python3
"""Build a narrow servo-angle trial configuration, independent of unmeasured URDF zeros."""
import argparse,hashlib,json,math,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def generate(profile,output,enable=False):
    if profile['channels']!=[0,1,2,5,6,7]:raise ValueError('Unexpected channel map')
    if enable and (profile.get('power_verified') is not True or profile.get('oe_wired_verified') is not True or not 4.8<=profile.get('servo_rail_voltage',0)<=6.6):
        raise ValueError('Motor trial stays locked: verify appropriate servo power and GPIO23/OE wiring first')
    if profile['servo_range_degrees']!=[0,180] or profile['home_degrees']!=90:raise ValueError('Only confirmed 180-degree servos at 90-degree startup supported')
    ranges=profile['pulse_ranges_us']
    if ranges!=[[500,2500]]*5+[[1000,2300]]:raise ValueError('Pulse map differs from confirmed trial profile')
    output.mkdir(parents=True,exist_ok=True)
    shutil.copy2(ROOT/'firmware/arm_controller/arm_controller.ino',output/'arm_controller.ino')
    ident='TRIAL_'+hashlib.sha256(json.dumps(profile,sort_keys=True).encode()).hexdigest()[:16]
    low,high=88.,92.
    lines=['#pragma once','// Servo-angle bench trial: NOT robot joint coordinates.',
        f'constexpr bool HARDWARE_CALIBRATED = {str(enable).lower()};',
        'constexpr int SDA_PIN=21, SCL_PIN=22, OE_PIN=23;',
        f'constexpr bool OE_WIRED = {str(profile.get("oe_wired_verified") is True).lower()};',
        'constexpr unsigned char PCA_ADDRESS=0x40;', 'constexpr int JOINT_COUNT=6;',
        f'constexpr const char* CONFIG_ID="{ident}";']
    values={'CHANNEL':profile['channels'],'RAD_LOW':[math.radians(low)]*6,'RAD_HIGH':[math.radians(high)]*6,
            'PULSE_AT_LOW':[a+(b-a)*low/180 for a,b in ranges],
            'PULSE_AT_HIGH':[a+(b-a)*high/180 for a,b in ranges],
            'INITIAL_RAD':[math.pi/2]*6,'MAX_RAD_PER_SEC':[math.radians(2)]*6}
    for key,v in values.items():lines.append(f'constexpr {"int" if key=="CHANNEL" else "float"} {key}[JOINT_COUNT]={{'+','.join(map(str,v))+'};')
    (output/'hardware_config.h').write_text('\n'.join(lines)+'\n')
    (output.parent/'trial_manifest.json').write_text(json.dumps({'enabled':enable,'config_id':ident,'servo_coordinates_only':True},indent=2)+'\n')
    print('Generated '+('ENABLED' if enable else 'LOCKED')+' motor trial at '+str(output))
    return ident

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--enable',action='store_true');a=p.parse_args()
    try:generate(json.loads((ROOT/'hardware/motor_trial.json').read_text()),ROOT/'hardware/motor_trial/arm_controller',a.enable)
    except ValueError as exc:raise SystemExit(str(exc))
