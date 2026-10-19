"""Exercise launcher failures and process ownership without ROS, X11 or a simulator."""
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
import uuid

NAV = Path(__file__).resolve().parents[1]
PROJECT = NAV.parents[2]
TOOLS = PROJECT / 'baseline/g1_mapping_nav/tools'
DEMO = NAV / 'scripts/run_gui_console_demo.sh'

STUB = r'''import json, os, pathlib, signal, sys, time
name=pathlib.Path(sys.argv[0]).name
args=sys.argv[1:]
trace=pathlib.Path(os.environ['STUB_TRACE'])
with trace.open('a') as f: f.write(json.dumps({'name':name,'args':args,'display':os.getenv('DISPLAY')})+'\n')
def stay():
 signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
 signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
 while True: time.sleep(.1)
state=pathlib.Path(os.environ['STUB_STATE'])
mode=os.getenv('STUB_MODE','ok')
if name in ('gui-wrapper','ros-wrapper'):
 body=args[-1]
 if 'topic echo' in body:
  if mode!='not-ready': print('data: '+(state.read_text() if state.exists() else 'standby'))
 elif 'topic pub' in body:
  if mode=='publish-fails': sys.exit(1)
  if mode!='reject-explore': state.write_text('exploring')
 elif 'ros2 launch' in body:
  print('Configured and activated joint_group_controller',flush=True);stay()
 else: stay()
elif name=='python3':
 if args and args[0].endswith('/web_console/server.py'): stay()
 os.execv(sys.executable,[sys.executable,*args])
elif name=='xwininfo': print('0x123 "Gazebo":')
elif name=='ss' and mode=='occupied-many':
 print('LISTEN 0 10 127.0.0.1:18765 0.0.0.0:*')
 for _ in range(10000): print('LISTEN 0 10 127.0.0.1:20000 0.0.0.0:*')
elif name=='curl': print('{"window":{"available": true}}')
elif name=='sleep': time.sleep(.01)
elif name=='timeout': os.execvp(args[1],args[1:])
elif name=='ros2':
 if args[:2]==['topic','info']: print('Publisher count: 1')
 if 'service' in args:
  status=json.loads(state.read_text())
  if 'list_controllers' in args[2]:
   print(', '.join("name='%s', state='%s'"%x for x in status.items()))
  elif mode!='activation-fails':
   for controller in status:
    if controller in args[-1]: status[controller]='inactive' if 'configure_controller' in args[2] else 'active'
   state.write_text(json.dumps(status))
'''


class ScriptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='go2-script-tests-')
        # unittest runs cleanups after tearDown, in reverse registration order.
        # Keep the private process helper until all launchers have been stopped.
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        # A private checkout skeleton avoids depending on the developer's .env.
        project=self.root/'project'
        self.demo=project/'external/go2_real_robot/qwen_robotnav/scripts/run_gui_console_demo.sh'
        self.demo.parent.mkdir(parents=True)
        shutil.copyfile(DEMO,self.demo)
        helper=project/'baseline/g1_mapping_nav/tools/owned_processes.py'
        helper.parent.mkdir(parents=True)
        shutil.copyfile(TOOLS/'owned_processes.py',helper)
        (project/'.env').touch()
        self.bin = self.root / 'bin'; self.bin.mkdir()
        for name in ['gui-wrapper','ros-wrapper','python3','xwininfo','curl','ss','x11vnc','xdotool','sleep','timeout','ros2']:
            p = self.bin/name
            p.write_text('#!'+sys.executable+'\n'+STUB);p.chmod(0o755)
        self.out = self.root/'out'; self.out.mkdir()
        (self.out/'progress_install').mkdir();(self.out/'gazebo_nav2.launch.py').touch()
        self.env = dict(os.environ, PATH=str(self.bin)+':'+os.environ['PATH'],
                        STUB_TRACE=str(self.root/'trace'), STUB_STATE=str(self.root/'state'),
                        DEMO_OUT=str(self.out), G1_ROS_WRAPPER=str(self.bin/'ros-wrapper'),
                        G1_GPU_WRAPPER=str(self.bin/'gui-wrapper'), DEMO_READY_TIMEOUT_S='1',
                        DEMO_CONSOLE_PORT='18765', DEMO_WS_PORT='18766', DEMO_VNC_PORT='15909',
                        DEMO_DISPLAY=':98', DISPLAY=':777')

    def records(self):
        p=self.root/'trace'
        return [json.loads(line) for line in p.read_text().splitlines()] if p.exists() else []

    def launch(self, mode='ok', explore=False):
        env=dict(self.env,STUB_MODE=mode)
        log=self.root/'launcher.log'
        with log.open('w') as stream:
            child=subprocess.Popen(['bash',str(self.demo),*(['--explore'] if explore else [])],
                                   env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        self.addCleanup(self.stop,child)
        return child,log

    @staticmethod
    def stop(child):
        if child.poll() is None:
            child.terminate()
            try: child.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait()

    def test_custom_ports_and_display_reach_components(self):
        child,log=self.launch()
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and child.poll() is None:
            if 'Demo ready:' in log.read_text():break
            time.sleep(.05)
        self.assertIn('Demo ready: http://127.0.0.1:18765',log.read_text())
        records=self.records()
        guis=[r for r in records if r['name']=='gui-wrapper']
        self.assertEqual(len(guis),2)
        self.assertTrue(all(r['display']==':98' for r in guis))
        console=next(r for r in records if r['name']=='python3' and r['args'][0].endswith('/server.py'))
        for flag,value in [('--port','18765'),('--ws-port','18766'),('--vnc-port','15909'),('--display',':98')]:
            self.assertEqual(console['args'][console['args'].index(flag)+1],value)
        self.stop(child)
        self.assertFalse((self.out/'demo.pid').exists())

    def test_registered_cleanup_stops_private_children_before_removing_checkout(self):
        child,log=self.launch()
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and child.poll() is None:
            if 'Demo ready:' in log.read_text():break
            time.sleep(.05)
        self.assertIn('Demo ready:',log.read_text())
        prefix=str(self.bin)+'/'
        owned=[]
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():continue
            try:args=(p/'cmdline').read_bytes().split(b'\0')
            except OSError:continue
            if any(a.decode(errors='replace').startswith(prefix) for a in args):
                owned.append(int(p.name))
        self.assertGreaterEqual(len(owned),3)
        self.doCleanups()
        self.assertFalse(self.root.exists())
        self.assertIsNotNone(child.poll())
        for pid in owned:
            stat=Path(f'/proc/{pid}/stat')
            self.assertTrue(not stat.exists() or stat.read_text().rsplit(')',1)[1].split()[0]=='Z',
                            f'private test process {pid} leaked')

    def test_busy_port_is_detected_with_large_socket_listing(self):
        child,log=self.launch('occupied-many')
        self.assertNotEqual(child.wait(timeout=5),0)
        self.assertIn('already in use',log.read_text())
        self.assertFalse(any(r['name']=='gui-wrapper' for r in self.records()))

    def test_standby_timeout_never_reports_ready_or_starts_console(self):
        child,log=self.launch('not-ready')
        self.assertNotEqual(child.wait(timeout=15),0)
        self.assertIn('did not reach standby',log.read_text())
        self.assertNotIn('Demo ready:',log.read_text())
        self.assertFalse(any(r['name']=='python3' and r['args'][0].endswith('/server.py') for r in self.records()))
        self.assertFalse((self.out/'demo.pid').exists())

    def test_publish_failure_is_not_hidden(self):
        child,log=self.launch('publish-fails',True)
        self.assertNotEqual(child.wait(timeout=15),0)
        self.assertIn('EXPLORE publish failed',log.read_text())
        self.assertNotIn('Demo ready:',log.read_text())

    def test_published_but_rejected_explore_never_reports_ready(self):
        child,log=self.launch('reject-explore',True)
        self.assertNotEqual(child.wait(timeout=15),0)
        self.assertIn('did not reach exploring',log.read_text())
        self.assertNotIn('Demo ready:',log.read_text())

    def test_explore_requires_observed_state_transition(self):
        child,log=self.launch('ok',True)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and child.poll() is None:
            if 'Demo ready:' in log.read_text():break
            time.sleep(.05)
        self.assertIn('Demo ready:',log.read_text())
        self.assertEqual((self.root/'state').read_text(),'exploring')

    def check_controllers(self, states, mode='ok'):
        (self.root/'state').write_text(json.dumps(states))
        script='source "$1"; require_active_controllers "$2"'
        return subprocess.run(['bash','-c',script,'_',str(TOOLS/'controller_check.sh'),str(self.out)],
                              env=dict(self.env,STUB_MODE=mode),text=True,capture_output=True,timeout=5)

    def test_missing_controller_is_rejected(self):
        result=self.check_controllers({})
        self.assertNotEqual(result.returncode,0)
        self.assertIn('无法确认',result.stderr)

    def test_both_controllers_are_configured_and_verified(self):
        result=self.check_controllers({'joint_group_controller':'unconfigured','joint_state_broadcaster':'unconfigured'})
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(set(json.loads((self.root/'state').read_text()).values()),{'active'})

    def test_activation_reply_without_state_change_is_rejected(self):
        result=self.check_controllers({'joint_group_controller':'inactive','joint_state_broadcaster':'active'},'activation-fails')
        self.assertNotEqual(result.returncode,0)

    def test_already_active_controllers_need_no_switch(self):
        result=self.check_controllers({'joint_group_controller':'active','joint_state_broadcaster':'active'})
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertFalse(any('switch_controller' in ' '.join(r['args']) for r in self.records()))

    def test_missing_broadcaster_is_rejected(self):
        self.assertNotEqual(self.check_controllers({'joint_group_controller':'active'}).returncode,0)

    def test_failed_preflight_preserves_other_sessions(self):
        canary=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)',
                                 'sync_slam_toolbox_node cmd_vel_scaler.py gt_relay.py ros2 bag record'],start_new_session=True)
        try:
            (self.root/'state').write_text('{}')
            for script,args in [('slam_e2e_check.sh',['3',str(self.out/'bag')]),('walk_test.sh',['3','0.02',str(self.out/'bag')]),('start_nav2_isolated.sh',['--slam'])]:
                result=subprocess.run(['bash',str(TOOLS/script),*args],env=dict(self.env,G1_LOG_DIR=str(self.out)),
                                      text=True,capture_output=True,timeout=6)
                self.assertNotEqual(result.returncode,0,result.stdout)
                self.assertIsNone(canary.poll(),script+' killed an unrelated process')
        finally:
            canary.terminate();canary.wait()

    def test_cleanup_includes_descendant_with_its_own_session(self):
        pidfile=self.root/'child.pid'
        code="import subprocess,time,pathlib,sys; p=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],start_new_session=True);pathlib.Path(sys.argv[1]).write_text(str(p.pid));time.sleep(60)"
        parent=subprocess.Popen([sys.executable,'-c',code,str(pidfile)],start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        try:
            deadline=time.monotonic()+3
            while not pidfile.exists() and time.monotonic()<deadline:time.sleep(.02)
            descendant=int(pidfile.read_text())
            subprocess.run([sys.executable,str(TOOLS/'owned_processes.py'),'--grace','0.1',str(parent.pid)],check=True,timeout=4)
            parent.wait(timeout=2)
            stat=Path(f'/proc/{descendant}/stat')
            self.assertTrue(not stat.exists() or stat.read_text().rsplit(')',1)[1].split()[0]=='Z')
        finally:
            if parent.poll() is None:parent.kill();parent.wait()
            if pidfile.exists():
                try:os.kill(int(pidfile.read_text()),signal.SIGKILL)
                except ProcessLookupError:pass

    def test_cleanup_retains_ownership_after_wrapper_exits(self):
        pidfile=self.root/'orphan.pid'
        code="import subprocess,pathlib,sys; p=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)']);pathlib.Path(sys.argv[1]).write_text(str(p.pid))"
        wrapper=subprocess.Popen([sys.executable,'-c',code,str(pidfile)],start_new_session=True,
                                 stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        wrapper.wait(timeout=3)
        descendant=int(pidfile.read_text())
        canary=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],start_new_session=True)
        try:
            subprocess.run([sys.executable,str(TOOLS/'owned_processes.py'),'--grace','0.1',str(wrapper.pid)],check=True,timeout=4)
            stat=Path(f'/proc/{descendant}/stat')
            self.assertTrue(not stat.exists() or stat.read_text().rsplit(')',1)[1].split()[0]=='Z')
            self.assertIsNone(canary.poll())
        finally:
            canary.terminate();canary.wait()
            try:os.kill(descendant,signal.SIGKILL)
            except ProcessLookupError:pass

    def test_private_partition_retains_detached_orphan_without_stopping_other_sessions(self):
        partition='runtime_'+uuid.uuid4().hex[:12]
        pidfile=self.root/'detached.pid'
        code="import subprocess,pathlib,sys; p=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],start_new_session=True);pathlib.Path(sys.argv[1]).write_text(str(p.pid))"
        wrapper=subprocess.Popen([sys.executable,'-c',code,str(pidfile)],env=dict(os.environ,IGN_PARTITION=partition),
                                 start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        wrapper.wait(timeout=3);descendant=int(pidfile.read_text())
        canary=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],start_new_session=True,
                                env=dict(os.environ,IGN_PARTITION='runtime_'+uuid.uuid4().hex[:12]))
        try:
            subprocess.run([sys.executable,str(TOOLS/'owned_processes.py'),'--grace','.1','--partition',partition],check=True,timeout=4)
            stat=Path(f'/proc/{descendant}/stat')
            self.assertTrue(not stat.exists() or stat.read_text().rsplit(')',1)[1].split()[0]=='Z')
            self.assertIsNone(canary.poll())
        finally:
            canary.terminate();canary.wait()
            try:os.kill(descendant,signal.SIGKILL)
            except ProcessLookupError:pass


if __name__ == '__main__':
    unittest.main()
