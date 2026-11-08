import argparse,json,tkinter as tk
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('run_dir',type=Path);a=p.parse_args()
r=tk.Tk();r.title('Autonomous sorting — simulation');r.geometry('640x280')
text=tk.StringVar(value='Starting Gazebo and MoveIt…')
tk.Label(r,text='AUTONOMOUS SORTING — GAZEBO ONLY',font=('Sans',16,'bold')).pack(pady=14)
tk.Label(r,textvariable=text,wraplength=600,font=('Sans',12)).pack(pady=10)
def stop(*_):
 (ROOT/'autonomous/STOP').touch();text.set('STOP requested. No automatic restart.')
tk.Button(r,text='STOP — O',command=stop,bg='#b00020',fg='white',font=('Sans',18)).pack()
tk.Label(r,text='O works in this window. This controller does not access real motors.').pack(pady=8)
r.bind('<KeyPress-o>',stop);r.bind('<KeyPress-O>',stop)
def poll():
 try:
  s=json.loads((a.run_dir/'status.json').read_text());text.set(s['phase']+'\n'+str(s.get('object',s.get('reason',s.get('counts','')))))
 except (OSError,ValueError):pass
 r.after(200,poll)
r.protocol('WM_DELETE_WINDOW',stop);poll()
try:r.mainloop()
except KeyboardInterrupt:pass
