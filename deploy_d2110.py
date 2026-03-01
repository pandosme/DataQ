#!/usr/bin/env python3
import subprocess, time, sys

host = '10.13.8.109'
user = 'nodered'
pw = 'rednode'
eap = '/home/fred/development/Radar/DataQ_Radar_2_2_0_armv7hf.eap'

def curl(*args):
    cmd = ['curl', '--silent', '--digest', '-u', f'{user}:{pw}'] + list(args)
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(r.stdout.strip() or r.stderr.strip())
    return r.returncode

print('Uploading...')
rc = curl('-F', f'packfil=@{eap};type=application/octet-stream',
          f'http://{host}/axis-cgi/applications/upload.cgi')
if rc != 0:
    print('Upload failed')
    sys.exit(1)

time.sleep(3)
print('Starting...')
curl(f'http://{host}/axis-cgi/applications/control.cgi?action=start&package=DataQ')

time.sleep(5)
print('Checking logs...')
curl(f'http://{host}/axis-cgi/admin/systemlog.cgi?appname=DataQ')
