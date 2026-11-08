"""Tabular Q-learning for sorting ORDER only; MoveIt controls collision-checked motion."""
import json,random
from pathlib import Path
REVISION='sort_order_zy_home90_v1'
ACTIONS=('nearest','black_first','white_first')
class OrderPolicy:
 def __init__(self,path,learn=False,seed=0):
  self.path=Path(path);self.learn=learn;self.rng=random.Random(seed);self.q={};self.updates=0
  if self.path.exists():
   d=json.loads(self.path.read_text())
   if d.get('revision')!=REVISION:raise ValueError('Obsolete sorting policy; use a new checkpoint')
   self.q=d['q'];self.updates=d['updates']
 def state(self,items,xy):
  return f"{sum(x['color']=='black' for x in items)}:{sum(x['color']=='white' for x in items)}:{int(xy[1]>=0)}"
 def choose(self,items,xy):
  if not items:raise ValueError('No candidate')
  state=self.state(items,xy);values=self.q.get(state,[0.]*3)
  a=self.rng.randrange(3) if self.learn and self.rng.random()<.2 else max(range(3),key=lambda i:values[i])
  color=('black' if a==1 else 'white' if a==2 else None)
  pool=[x for x in items if x['color']==color] or items
  selected=min(pool,key=lambda x:((x['xyz'][0]-xy[0])**2+(x['xyz'][1]-xy[1])**2,x['name']))
  return selected,(state,a)
 def update(self,decision,reward,remaining,xy):
  if not self.learn:return
  state,a=decision;row=self.q.setdefault(state,[0.]*3)
  future=max(self.q.get(self.state(remaining,xy),[0.]*3)) if remaining else 0.
  row[a]+=.2*(reward+.95*future-row[a]);self.updates+=1
  self.path.parent.mkdir(parents=True,exist_ok=True)
  tmp=self.path.with_suffix('.tmp');tmp.write_text(json.dumps({'revision':REVISION,'scope':'object selection order; not grasp control','updates':self.updates,'q':self.q},indent=2)+'\n');tmp.replace(self.path)
