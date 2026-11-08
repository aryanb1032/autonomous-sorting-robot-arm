#!/usr/bin/env python3
"""Generate matching firmware/ROS files ONLY from complete measured calibration."""
import argparse,hashlib,json,math,shutil,sys
from pathlib import Path
import xml.etree.ElementTree as E
import xacro,yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/arm_sorter'))
from arm_sorter.jaw_mapping import JawMapping
sys.path.insert(0,str(ROOT/'model'))
from geometry import GEOMETRY,REVISION
NAMES=[f'joint_{i}' for i in range(1,6)]+['gripper_joint']


def validate(p):
    for flag in ['calibration_confirmed','servo_power_verified','oe_wired_verified']:
        if p.get(flag) is not True:raise ValueError(f'{flag} must reflect an actual completed check')
    def number(value,label):
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):raise ValueError(f'{label}: finite measured value required')
    v=p.get('servo_rail_voltage');number(v,'servo_rail_voltage')
    if not 4.8<=v<=6.6:raise ValueError('MG996R rail must be within its verified rating; supplied 7 V cannot be used directly')
    if p.get('sda')!=21 or p.get('scl')!=22:raise ValueError('This wiring profile requires SDA21/SCL22')
    if p.get('oe_pin')!=23:raise ValueError('This profile requires OE on GPIO23 with 3.3 V pull-up')
    address=p.get('pca_address')
    if type(address)!=int or not 0x40<=address<=0x77:raise ValueError('Confirm PCA9685 I2C address')
    rows=p.get('joints',[])
    if [j.get('name') for j in rows]!=NAMES:raise ValueError('Joint names/order must be base to tool then gripper_joint')
    channels=[]
    for j in rows:
        if type(j.get('channel'))!=int or not 0<=j['channel']<=15:raise ValueError(f"Channel missing/invalid: {j['name']}")
        channels.append(j['channel'])
        for k in ['rad_low','rad_high','pulse_at_low_us','pulse_at_high_us','initial_rad','max_rad_per_sec']:number(j.get(k),j['name']+'.'+k)
        if not j['rad_low']<j['rad_high'] or not j['rad_low']<=j['initial_rad']<=j['rad_high']:raise ValueError('Invalid measured angle range/initial pose')
        if not 0<j['max_rad_per_sec']<=1:raise ValueError('Initial commissioning speed must be >0 and <=1 rad/s')
        if any(not 400<=j[k]<=2600 for k in ['pulse_at_low_us','pulse_at_high_us']) or j['pulse_at_low_us']==j['pulse_at_high_us']:raise ValueError('Invalid measured PWM endpoints')
    if len(set(channels))!=6:raise ValueError('Servo channels must be unique')
    if p.get('geometry_revision')!=REVISION:raise ValueError('Hardware profile uses an obsolete geometry revision; recalibrate and regenerate')
    if p.get('gripper_model')!='mirrored_hinge_proxy':raise ValueError('Select current hinged gripper model')
    if p.get('gripper_finger_length_m')!=GEOMETRY['gripper_motor_to_tip']:raise ValueError('Finger length must match geometry')
    jaw=JawMapping(p.get('gripper_width_table'),p['gripper_finger_length_m'])
    for angle in jaw.table[:,0]:jaw.to_joint(float(angle))
    if not math.isclose(jaw.table[0,0],rows[-1]['rad_low']) or not math.isclose(jaw.table[-1,0],rows[-1]['rad_high']):raise ValueError('Jaw table must cover the entire gripper actuator angle range')
    return jaw


def generate(p,out):
    jaw=validate(p);out.mkdir(parents=True,exist_ok=True)
    rows=p['joints'];fingerprint=hashlib.sha256(json.dumps(p,sort_keys=True).encode()).hexdigest()[:20]
    sketch=out/'arm_controller';sketch.mkdir(exist_ok=True)
    shutil.copy2(ROOT/'firmware/arm_controller/arm_controller.ino',sketch/'arm_controller.ino')
    header=['#pragma once','// Generated from measured hardware profile; no guessed channel or pulse values.',
            'constexpr bool HARDWARE_CALIBRATED = true;','constexpr int SDA_PIN = 21;','constexpr int SCL_PIN = 22;',
            'constexpr int OE_PIN = 23;','constexpr bool OE_WIRED = true;',f"constexpr unsigned char PCA_ADDRESS = {p['pca_address']};",
            'constexpr int JOINT_COUNT = 6;',f'constexpr const char* CONFIG_ID = "{fingerprint}";']
    for field,name,typ in [('channel','CHANNEL','int'),('rad_low','RAD_LOW','float'),('rad_high','RAD_HIGH','float'),('pulse_at_low_us','PULSE_AT_LOW','float'),('pulse_at_high_us','PULSE_AT_HIGH','float'),('initial_rad','INITIAL_RAD','float'),('max_rad_per_sec','MAX_RAD_PER_SEC','float')]:
        header.append(f'constexpr {typ} {name}[JOINT_COUNT] = '+'{'+','.join(str(j[field]) for j in rows)+'};')
    (sketch/'hardware_config.h').write_text('\n'.join(header)+'\n')
    cfg={'calibrated':True,'config_id':fingerprint,'port':p.get('port') or '', 'joint_names':NAMES,
         'initial_rad':[j['initial_rad'] for j in rows],'limits':{j['name']:[j['rad_low'],j['rad_high']] for j in rows},
         'max_velocity':{j['name']:j['max_rad_per_sec'] for j in rows},'gripper_width_table':p['gripper_width_table'], 'geometry_revision':REVISION,
         'gripper_model_joint':'gripper_hinge_joint','gripper_mimic_joint':'gripper_hinge_mimic',
         'gripper_finger_length_m':p['gripper_finger_length_m']}
    (out/'bridge.json').write_text(json.dumps(cfg,indent=2)+'\n')
    (out/'profile.json').write_text(json.dumps(p,indent=2)+'\n')
    pkg=ROOT/'ros2_ws/src/arm_simulation'
    robot=E.fromstring(xacro.process_file(str(pkg/'urdf/provisional_arm.urdf.xacro')).toxml())
    robot.set('name','calibrated_arm')
    for element in list(robot):
        if element.tag in ['ros2_control','gazebo']:robot.remove(element)
    mapping={f'sim_joint_{i}':f'joint_{i}' for i in range(1,6)}
    mapping.update(sim_finger_joint='gripper_hinge_joint',sim_finger_mimic='gripper_hinge_mimic')
    def rename(tree):
        for e in tree.iter():
            for k,v in list(e.attrib.items()):
                if v in mapping:e.set(k,mapping[v])
    rename(robot)
    for row in rows[:-1]:
        limit=robot.find(f"joint[@name='{row['name']}']/limit")
        limit.set('lower',str(row['rad_low']));limit.set('upper',str(row['rad_high']));limit.set('velocity',str(row['max_rad_per_sec']))
    travel=math.asin((jaw.maximum-jaw.minimum)/(2*p['gripper_finger_length_m']))
    slide_speed=min(abs(float((b[1]-a[1])/(b[0]-a[0])))/2 for a,b in zip(jaw.table,jaw.table[1:]))*rows[-1]['max_rad_per_sec']/p['gripper_finger_length_m']
    for name,sign in [('gripper_hinge_joint',1),('gripper_hinge_mimic',-1)]:
        j=robot.find(f"joint[@name='{name}']");j.find('limit').set('lower','0');j.find('limit').set('upper',str(travel));j.find('limit').set('velocity',str(slide_speed))
        j.find('origin').set('xyz',f"0 {sign*(jaw.minimum/2+.004)} {GEOMETRY['motor5_to_gripper_motor']}")
    E.indent(robot);E.ElementTree(robot).write(out/'real_arm.urdf',encoding='unicode',xml_declaration=True)
    srdf=E.parse(pkg/'config/provisional_arm.srdf');srdf.getroot().set('name','calibrated_arm');rename(srdf.getroot())
    for state in list(srdf.getroot().findall('group_state')):srdf.getroot().remove(state)
    state=E.SubElement(srdf.getroot(),'group_state',name='calibrated_initial',group='arm')
    for row in rows[:-1]:E.SubElement(state,'joint',name=row['name'],value=str(row['initial_rad']))
    E.indent(srdf);srdf.write(out/'real_arm.srdf',encoding='unicode',xml_declaration=True)
    limits={j['name']:{'has_velocity_limits':True,'max_velocity':j['max_rad_per_sec'],'has_acceleration_limits':True,'max_acceleration':j['max_rad_per_sec']} for j in rows[:-1]}
    limits['gripper_hinge_joint']={'has_velocity_limits':True,'max_velocity':slide_speed,'has_acceleration_limits':True,'max_acceleration':slide_speed}
    (out/'joint_limits.yaml').write_text(yaml.safe_dump({'joint_limits':limits}))
    mc=yaml.safe_load((ROOT/'ros2_ws/src/arm_sorter/config/moveit_controllers.yaml').read_text())
    mc['moveit_simple_controller_manager']['gripper_controller']['joints']=['gripper_hinge_joint']
    (out/'moveit_controllers.yaml').write_text(yaml.safe_dump(mc))
    print('Generated matching firmware, bridge and provisional model at',out)
    print('CONFIG_ID:',fingerprint,' Model/scene must still be physically verified before real execution.')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('profile',type=Path);p.add_argument('--output',type=Path,default=Path(__file__).parent/'generated');a=p.parse_args()
    try:generate(json.loads(a.profile.read_text()),a.output)
    except (ValueError,TypeError,KeyError) as exc:raise SystemExit('NOT READY: '+str(exc))
