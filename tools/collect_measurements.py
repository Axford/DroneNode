#!/usr/bin/env python3
"""Read numeric local diagnostics only; never writes parameters or controls a device."""
import argparse
import ipaddress
import json
import os
from pathlib import Path
import time
import urllib.parse
import urllib.request

PHASES={'request','dnsTcpTlsOrReuse','encodeChunk','writeChunk','response','report'}
NUMBERS=('uptimeMs','iterations','maxGapMs','overruns','sequence','overwritten','freeHeap','minHeap','largestBlock','encodeCalls','encodeUs','encodeMaxUs','writeCalls','writeUs','writeMaxUs')
EVENT_NUMBERS=('sequence','atMs','elapsedUs','freeHeap','minHeap','largestBlock','stackBytes')

def numeric(row, names):
    if not isinstance(row,dict) or any(type(row.get(name)) is not int or not 0<=row[name]<=0xffffffff for name in names):
        raise ValueError('invalid numeric measurement')
    return {name:row[name] for name in names}

def sanitize(row):
    if row.get('schemaVersion')!=1 or not isinstance(row.get('events'),list) or len(row['events'])>64:
        raise ValueError('unsupported measurement snapshot')
    result=numeric(row,NUMBERS);result['events']=[]
    for event in row['events']:
        clean=numeric(event,EVENT_NUMBERS)
        if event.get('phase') not in PHASES or type(event.get('end')) is not bool:
            raise ValueError('invalid phase measurement')
        clean.update(phase=event['phase'],end=event['end']);result['events'].append(clean)
    return result

class Cursor:
    def __init__(self):
        self.sequence=None
        self.uptime=None
    def consume(self,snapshot):
        row=sanitize(snapshot);sequence=row['sequence']
        reset=self.uptime is not None and (row['uptimeMs']<self.uptime or sequence<self.sequence)
        previous=0 if reset else self.sequence
        row['resetObserved']=reset
        # The first snapshot defines the baseline; prior events are not this run.
        events=[] if previous is None else [e for e in row['events'] if e['sequence']>previous]
        row['missingEvents']=0 if previous is None else max(0,sequence-previous-len(events))
        row['events']=events;self.sequence=sequence;self.uptime=row['uptimeMs']
        return row

def local_url(value):
    parsed=urllib.parse.urlsplit(value)
    try:
        address=ipaddress.ip_address(parsed.hostname)
        valid=address.is_private or address.is_loopback
    except ValueError:
        valid=False
    if parsed.scheme!='http' or not valid or parsed.username or parsed.password or parsed.query or parsed.fragment or parsed.path not in ('','/'):
        raise argparse.ArgumentTypeError('use an explicit local HTTP IP origin without credentials or path')
    return value.rstrip('/')+'/measurements'

def collect(url,output,seconds,interval):
    cursor=Cursor();start=time.monotonic();samples=missing=resets=unreachable=0
    # Avoid proxies and redirects: the selected local endpoint is the whole scope.
    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self,*_): return None
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect())
    fd=os.open(output,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(fd,'w') as stream:
        while time.monotonic()-start<seconds:
            try:
                with opener.open(url,timeout=2) as response:
                    raw=response.read(65537)
                    if len(raw)>65536: raise ValueError('measurement response exceeds limit')
            except OSError:
                row={'unreachable':True};unreachable+=1
            else:
                row=cursor.consume(json.loads(raw))
                samples+=1;missing+=row['missingEvents'];resets+=row['resetObserved']
            row['collectorElapsedMs']=int((time.monotonic()-start)*1000)
            stream.write(json.dumps(row,separators=(',',':'))+'\n');stream.flush()
            time.sleep(max(0,min(interval,seconds-(time.monotonic()-start))))
    return {'samples':samples,'missingEvents':missing,'resets':resets,'unreachableSamples':unreachable}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--origin',required=True,type=local_url)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--seconds',type=float,default=60)
    parser.add_argument('--interval',type=float,default=0.5)
    args=parser.parse_args()
    if not 1<=args.seconds<=1200 or not 0.25<=args.interval<=5:
        parser.error('duration must be1–1200seconds and interval0.25–5seconds')
    try: print(json.dumps(collect(args.origin,args.output,args.seconds,args.interval)))
    except (OSError,ValueError,TypeError,AttributeError): parser.exit(1,'Measurement collection failed; retain partial numeric evidence and inspect local reachability/schema.\n')
if __name__=='__main__': main()
