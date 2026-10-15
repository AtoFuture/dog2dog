"""Plot an online-SLAM frontier validation from its saved artifacts."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('output_dir',type=Path)
args=parser.parse_args();root=args.output_dir
report=json.loads((root/'report.json').read_text())
meta=json.loads((root/'map.json').read_text());grid=np.load(root/'map.npy')
with (root/'trajectory.csv').open() as stream:
    trace=np.asarray([[float(v) for v in row] for row in list(csv.reader(stream))[1:]])
start=np.asarray(report['truth_start'][:2]);home=np.asarray(report['home_map'][:2])
a=report['world_map_yaw_offset'];rotation=np.array([[np.cos(a),np.sin(a)],[-np.sin(a),np.cos(a)]])
xy=(trace[:,1:3]-start)@rotation.T+home
fig,axes=plt.subplots(1,2,figsize=(13,6),constrained_layout=True)
ax=axes[0];x,y=meta['origin'];res=meta['resolution']
ax.imshow(np.where(grid<0,.6,1-grid/100),origin='lower',cmap='gray',vmin=0,vmax=1,
          extent=[x,x+grid.shape[1]*res,y,y+grid.shape[0]*res])
ax.plot(xy[::10,0],xy[::10,1],color='tab:blue',linewidth=1,label='Truth route aligned at start')
for e in report['exploration_events']:
    if e['status']=='succeeded':
        p=e['target'];ax.scatter(p['x'],p['y'],marker='x',color='tab:red')
ax.scatter(*home,marker='s',color='tab:green',label='SLAM dock')
ax.set_xlabel('SLAM map x (m)');ax.set_ylabel('SLAM map y (m)');ax.legend(fontsize=8)
ax.set_title(f"Online SLAM | effective goals: {report.get('meaningful_frontier_goals',0)} ({report['successful_frontier_goals']} total reached)")
ax=axes[1];samples=np.asarray(report['map_samples'])
ax.plot(samples[:,0]-samples[0,0],samples[:,1],label='Known area')
ax.plot(samples[:,0]-samples[0,0],samples[:,2],label='Free area')
ax.set_xlabel('Wall time since first map (s)');ax.set_ylabel('Observed area (m²)')
ax.set_title('Observed map area; not a whole-site coverage percentage')
ax.grid(alpha=.3);ax.legend()
evaluation=root/'evaluation/bag_report.json'
if evaluation.exists():
    coverage=json.loads(evaluation.read_text())['metrics']['coverage']
    if coverage.get('available'):
        fig.suptitle(f"Reference free-space coverage: {coverage['percent']:.2f}% (offline registration)")
fig.savefig(root/'slam_frontier_summary.png',dpi=160)
print(root/'slam_frontier_summary.png')
