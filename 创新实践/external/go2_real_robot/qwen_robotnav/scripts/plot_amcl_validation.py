"""Plot a saved AMCL navigation validation; no ROS or API access required."""
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
args=parser.parse_args()
root=args.output_dir
report=json.loads((root/'report.json').read_text())
with (root/'trajectory.csv').open() as stream:
    trajectory=np.asarray([[float(v) for v in row] for row in list(csv.reader(stream))[1:]])
localization=json.loads((root/'localization.json').read_text())
amcl=np.asarray(localization['amcl'])
# Keep high-frequency Gazebo samples from obscuring the route.
bins={}
for row in trajectory:
    bins.setdefault(int((row[0]-trajectory[0,0])*5),[]).append(row[1:3])
xy=np.asarray([np.median(bins[key],axis=0) for key in sorted(bins)])
fig,axes=plt.subplots(1,2,figsize=(12,5),constrained_layout=True)
ax=axes[0]
ax.plot(xy[:,0],xy[:,1],label='Simulator truth')
if len(amcl):
    ax.plot(amcl[:,1],amcl[:,2],'.--',markersize=3,alpha=.7,label='AMCL estimate')
for cycle in report.get('cycles',[]):
    ax.scatter(*cycle['outbound_truth'][:2],marker='x',s=70,label=f"Cycle {cycle['cycle']} outbound")
ax.scatter(*report['truth_start'][:2],marker='s',s=60,label='Dock / start')
ax.set_aspect('equal',adjustable='datalim')
ax.set_xlabel('Map x (m)');ax.set_ylabel('Map y (m)');ax.grid(alpha=.3);ax.legend(fontsize=8)
ax.set_title('AMCL localization; ground truth used for evaluation')
ax=axes[1]
if report.get('cycles'):
    cycles=report['cycles']
    ax.bar([c['cycle'] for c in cycles],[c['home_error_m'] for c in cycles])
    ax.axhline(.4,linestyle='--',color='red',label='Test home error limit')
    ax.set_xticks([c['cycle'] for c in cycles]);ax.set_xlabel('Exploration / battery RETURN cycle')
    ax.set_ylabel('Truth distance from dock (m)');ax.legend()
else:
    ax.text(.05,.8,report.get('error','No completed return cycle'),wrap=True,transform=ax.transAxes)
ax.grid(axis='y',alpha=.3)
ax.set_title(f"{'PASS' if report['success'] else 'FAIL'} | completed {len(report.get('cycles',[]))} cycles\nEmergency guard interventions: {report['emergency_guard_interventions']}")
fig.savefig(root/'amcl_summary.png',dpi=160)
print(root/'amcl_summary.png')
