"""Actual firmware message checked-copy tests, no Arduino/device execution."""
import json, os, shutil, subprocess, tempfile, unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
class LinkValidationTests(unittest.TestCase):
 def test_actual_checked_copy(self):
  fixtures=json.loads((ROOT/'tests/fixtures/validation-vectors.json').read_text())
  with tempfile.TemporaryDirectory() as directory:
   work=Path(directory);shutil.copytree(ROOT/'tests/link_stubs',work/'src')
   for name in ('DroneLinkMsg.cpp','DroneLinkMsg.h'):shutil.copyfile(ROOT/'src'/name,work/'src'/name)
   lines=['#include "DroneLinkMsg.h"','#include <cassert>','#include <cstring>','int main(){ DroneLinkMsg msg; msg._msg={};']
   for kind in ('valid','invalid'):
    for case in fixtures[kind]:
     raw=bytes.fromhex(case['hex']);initial=list(raw) or [0]
     lines+=['{ const uint8_t bytes[]={'+','.join(map(str,initial))+'}; auto before=msg._msg;',
      ('assert(msg.tryParse(bytes,%d));' if kind=='valid' else 'assert(!msg.tryParse(bytes,%d));') % len(raw)]
     if kind=='valid':lines+=['assert(std::memcmp(&msg._msg,bytes,%d)==0);'%len(raw)]
     else:lines+=['assert(std::memcmp(&msg._msg,&before,sizeof before)==0);']
     lines+=['}']
   lines += ['for(unsigned type=0;type<8;type++)for(unsigned length=1;length<=16;length++){',
    'uint8_t bytes[21]={0,255,255,255,uint8_t(128|(type<<4)|(length-1))};',
    'bool valid = (type<1 || type>3 || length%4==0); auto before=msg._msg;',
    'assert(msg.tryParse(bytes,5+length)==valid);',
    'if(!valid)assert(std::memcmp(&msg._msg,&before,sizeof before)==0);',
    'else for(unsigned i=length;i<16;i++)assert(msg._msg.payload.uint8[i]==0);',
    '}', 'assert(!msg.tryParse(nullptr,6));','}'];(work/'src/check.cpp').write_text('\n'.join(lines))
   exe=str(work/'check');subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-fsanitize=address,undefined','-I',str(work/'src'),str(work/'src/check.cpp'),str(work/'src/DroneLinkMsg.cpp'),'-o',exe],check=True)
   subprocess.run([exe],check=True)
