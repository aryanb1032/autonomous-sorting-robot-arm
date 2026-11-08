"""Generate randomized, spaced pickups, fixed bins and per-object grasp plugins."""
from pathlib import Path
import argparse,math,json,random,secrets
import xml.etree.ElementTree as E
ROOT=Path(__file__).resolve().parents[1]
WORLDS=ROOT/'ros2_ws/src/arm_simulation/worlds'


def generate(seed,per_color=3,write=True):
    if not 1<=per_color<=3:raise ValueError('Use 1–3 boxes per colour; current bins have three slots')
    rng=random.Random(seed)
    tree=E.parse(WORLDS/'workcell.sdf');world=tree.getroot().find('world')
    for item in list(world.findall('model')):
        if item.get('name')!='ground':world.remove(item)
    parts=[]
    def solid(name,xyz,size,color):
        model=E.SubElement(world,'model',name=name)
        E.SubElement(model,'static').text='true'
        E.SubElement(model,'pose').text=' '.join(map(str,xyz))+' 0 0 0'
        link=E.SubElement(model,'link',name='body')
        for tag in ['collision','visual']:
            element=E.SubElement(link,tag,name=tag)
            E.SubElement(E.SubElement(E.SubElement(element,'geometry'),'box'),'size').text=' '.join(map(str,size))
            if tag=='visual':
                material=E.SubElement(element,'material')
                for prop in ['ambient','diffuse']:E.SubElement(material,prop).text=color
        parts.append(dict(name=name,xyz=xyz,size=size,color=list(map(float,color.split()))))
    layout={'seed':seed,'per_color':per_color,'objects':{},'bins':{},'static':parts}
    colors=['black']*per_color+['white']*per_color
    rng.shuffle(colors)
    # Reject complete left/right colour segregation, while preserving seeded randomness.
    while per_color>1 and len(set(colors[:per_color]))==1:rng.shuffle(colors)
    counts={'black':0,'white':0}
    count=len(colors)
    # Restart a crowded partial layout instead of getting stuck with five boxes.
    for attempt in range(100):
        points=[]
        for _ in range(1000):
            angle=rng.uniform(-.72,.72);radius=rng.uniform(.275,.335)
            candidate=(radius*math.cos(angle),radius*math.sin(angle),angle,radius)
            if all(math.hypot(candidate[0]-q[0],candidate[1]-q[1])>.065 for q in points):
                points.append(candidate)
                if len(points)==count:break
        if len(points)==count:break
    else:raise RuntimeError('Could not sample spaced ground positions; try another seed')
    points.sort(key=lambda q:q[2])
    for i,color in enumerate(colors):
        counts[color]+=1;name=f'{color}_box_{counts[color]:02d}'
        x,y,angle,radius=points[i];xy=[x,y]
        support='sorting_ground' # Actual floor pickup: no pedestal or plinth.
        yaw=rng.uniform(-.15,.15)
        model=E.parse(WORLDS/f'{color}_box.sdf').getroot().find('model');model.set('name',name)
        model.find('pose').text=' '.join(map(str,xy+[.011,0,0,yaw]))
        world.append(model)
        layout['objects'][name]={'color':color,'xyz':xy+[.01],'source_angle':angle,
                                'source_radius':radius,'yaw':yaw,'support':support}
    for color,sign,rgba in [('black',-1,'0.10 0.12 0.16 1'),('white',1,'0.85 0.85 0.9 1')]:
        angle=sign*1.1 # Separate fixed bins from the mixed pickup arc.
        bx,by=.3*math.cos(angle),.3*math.sin(angle)
        slots=[[bx+dx,by+dy,.015] for dx,dy in [(-.018,-.018),(.018,-.018),(0,.020)]]
        layout['bins'][color]={'centre':[bx,by,.015],'angle':angle,'inner_half_width':.045,'slots':slots}
        solid(f'{color}_bin_floor',[bx,by,.0025],[.10,.10,.005],rgba)
        for i,(dx,dy,sx,sy) in enumerate([(-.0475,0,.005,.1),(.0475,0,.005,.1),(0,-.0475,.09,.005),(0,.0475,.09,.005)]):
            solid(f'{color}_bin_wall_{i}',[bx+dx,by+dy,.025],[sx,sy,.04],rgba)
    if not write:return layout
    E.indent(tree,space='  ')
    # All outputs derive from the same object list, preventing plugin/name mismatches.
    tree.write(WORLDS/'sorting_workcell.sdf',encoding='unicode',xml_declaration=True)
    (ROOT/'sorting/layout.json').write_text(json.dumps(layout,indent=2)+'\n')
    plugins=E.Element('robot',{'xmlns:xacro':'http://www.ros.org/wiki/xacro'})
    for name in layout['objects']:
        plugin=E.SubElement(E.SubElement(plugins,'gazebo'),'plugin',filename='gz-sim-detachable-joint-system',name='gz::sim::systems::DetachableJoint')
        for tag,value in {'parent_link':'gripper_palm','child_model':name,'child_link':'body',
                          'attach_topic':f'/sorting/{name}/attach','detach_topic':f'/sorting/{name}/detach',
                          'output_topic':f'/sorting/{name}/state'}.items():
            E.SubElement(plugin,tag).text=value
    E.indent(plugins,space='  ')
    E.ElementTree(plugins).write(ROOT/'ros2_ws/src/arm_simulation/urdf/sorting_attachments.xacro',encoding='unicode',xml_declaration=True)
    return layout

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--seed',type=int);p.add_argument('--per-color',type=int,default=3,choices=[1,2,3]);a=p.parse_args()
    seed=a.seed if a.seed is not None else secrets.randbelow(2**31)
    layout=generate(seed,a.per_color)
    print(f'Generated {len(layout["objects"])} mixed boxes. Seed: {seed}',flush=True)
