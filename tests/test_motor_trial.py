import copy,json,sys
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'hardware'))
from prepare_motor_trial import generate

def profile():
    p=json.loads((ROOT/'hardware/motor_trial.json').read_text())
    p.update(power_verified=False,servo_rail_voltage=7.0,oe_wired_verified=False)
    return p

def test_current_power_and_oe_prevent_enable(tmp_path):
    with pytest.raises(ValueError):generate(profile(),tmp_path/'arm_controller',True)
    assert not (tmp_path/'arm_controller').exists()

@pytest.mark.parametrize('power,oe,voltage',[(False,True,6.),(True,False,6.),(True,True,7.)])
def test_independent_gates(tmp_path,power,oe,voltage):
    p=profile();p.update(power_verified=power,oe_wired_verified=oe,servo_rail_voltage=voltage)
    with pytest.raises(ValueError):generate(p,tmp_path/'arm_controller',True)

def test_locked_firmware_preserves_channels_and_does_not_enable(tmp_path):
    generate(profile(),tmp_path/'arm_controller')
    h=(tmp_path/'arm_controller/hardware_config.h').read_text()
    assert 'HARDWARE_CALIBRATED = false' in h
    assert 'CHANNEL[JOINT_COUNT]={0,1,2,5,6,7}' in h
    assert json.loads((tmp_path/'trial_manifest.json').read_text())['enabled'] is False
