"""Production measurement state and secret-safe collector behavior."""
from pathlib import Path
import os, subprocess, tempfile, unittest
ROOT=Path(__file__).resolve().parents[1]
class MeasurementTests(unittest.TestCase):
    def test_main_loop_and_bounded_phase_history(self):
        with tempfile.TemporaryDirectory() as work:
            source=Path(work)/'test.cpp'
            source.write_text('''#include "FirmwareMeasurements.h"
#include <cassert>
int main(){
 telemetry::Measurements m;
 m.loop(0xfffffff0); m.loop(0x10); m.loop(0x75); m.loop(0xd9);
 auto s=m.snapshot(); assert(s.iterations==4); assert(s.maxGapMs==101); assert(s.overruns==1);
 telemetry::Resources r={100,80,70,60};
 for(unsigned i=0;i<70;++i)m.record(telemetry::Phase::Connect,true,i,3,r);
 s=m.snapshot();assert(s.sequence==70);assert(s.count==64);assert(s.overwritten==6);
 assert(s.events[0].sequence==7);assert(s.events[63].sequence==70);
 assert(s.events[63].resources.stackBytes==60);
 m.record(telemetry::Phase::Encode,false,0,0,r);m.record(telemetry::Phase::Encode,true,0,7,r);
 m.record(telemetry::Phase::Encode,true,0,3,r);s=m.snapshot();assert(s.sequence==70);assert(s.encodeCalls==2&&s.encodeUs==10&&s.encodeMaxUs==7);
}''')
            exe=Path(work)/'test'
            subprocess.run([os.environ.get('CXX','c++'),'-std=c++11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(ROOT/'src'),str(source),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_collector_filters_secrets_and_marks_loss_and_reset(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('collector',ROOT/'tools/collect_measurements.py')
        collector=importlib.util.module_from_spec(spec);spec.loader.exec_module(collector)
        row=dict.fromkeys(collector.NUMBERS,0);row.update(schemaVersion=1,events=[],apiKey='MUST-NOT-RETAIN')
        cursor=collector.Cursor();self.assertNotIn('apiKey',cursor.consume(row))
        event=dict.fromkeys(collector.EVENT_NUMBERS,0);event.update(phase='report',end=True,sequence=3,apiKey='MUST-NOT-RETAIN')
        row.update(sequence=3,uptimeMs=100,events=[event])
        clean=cursor.consume(row);self.assertEqual(clean['missingEvents'],2);self.assertNotIn('apiKey',clean['events'][0])
        self.assertEqual(cursor.consume(row)['events'],[])
        row.update(sequence=0,uptimeMs=1,events=[]);self.assertTrue(cursor.consume(row)['resetObserved'])
        self.assertEqual(collector.local_url('http://192.168.1.32'),'http://192.168.1.32/measurements')
        for bad in ['https://staging.dronewright.com','http://8.8.8.8','http://user:secret@127.0.0.1','http://127.0.0.1/?key=secret']:
            with self.assertRaises(Exception):collector.local_url(bad)

    def test_actual_local_endpoint_serializes_collector_contract(self):
        import importlib.util,json
        spec=importlib.util.spec_from_file_location('collector',ROOT/'tools/collect_measurements.py')
        collector=importlib.util.module_from_spec(spec);spec.loader.exec_module(collector)
        with tempfile.TemporaryDirectory() as work:
            directory=Path(work)
            (directory/'Arduino.h').write_text('''#pragma once
#include <stdint.h>
#include <mutex>
using portMUX_TYPE=std::mutex;
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(p) (p)->lock()
#define portEXIT_CRITICAL(p) (p)->unlock()
inline uint32_t millis(){return 12;}
inline uint32_t uxTaskGetStackHighWaterMark(void*){return 321;}
struct Esp{uint32_t getFreeHeap(){return 10000;}uint32_t getMinFreeHeap(){return 8000;}};static Esp ESP;
''')
            (directory/'esp_heap_caps.h').write_text('#pragma once\n#define MALLOC_CAP_8BIT 1\ninline unsigned heap_caps_get_largest_free_block(int){return 7000;}')
            (directory/'ESPAsyncWebServer.h').write_text('''#pragma once
#include <functional>
#include <string>
#include <cstdarg>
#include <cstdio>
#define HTTP_GET 1
struct AsyncResponseStream {
 std::string text;
 void addHeader(const char*,const char*){}
 void printf(const char* format,...){char out[4096];va_list args;va_start(args,format);vsnprintf(out,sizeof out,format,args);va_end(args);text+=out;}
 void print(const char* value){text+=value;}
};
struct AsyncWebServerRequest {
 std::string text;
 AsyncResponseStream* beginResponseStream(const char*){return new AsyncResponseStream;}
 void send(int,const char*,const char*){}
 void send(AsyncResponseStream* response){text=response->text;delete response;}
};
struct AsyncWebServer {
 std::function<void(AsyncWebServerRequest*)> callback;
 void on(const char*,int,std::function<void(AsyncWebServerRequest*)> handler){callback=handler;}
};
''')
            (directory/'test.cpp').write_text('''#include "FirmwareMeasurements.h"
#include <ESPAsyncWebServer.h>
#include <iostream>
namespace telemetry {void mountMeasurements(AsyncWebServer&);}
int main(){telemetry::measureMainLoop(1);telemetry::measureMainLoop(103);
 telemetry::measurePhase(telemetry::Phase::Connect,false,0);
 telemetry::measurePhase(telemetry::Phase::Connect,true,123);
 telemetry::measurePhase(telemetry::Phase::Encode,true,8);
 AsyncWebServer server;telemetry::mountMeasurements(server);AsyncWebServerRequest request;server.callback(&request);std::cout<<request.text;
}''')
            exe=directory/'test'
            subprocess.run([os.environ.get('CXX','c++'),'-std=c++11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(directory),'-I',str(ROOT/'src'),str(ROOT/'src/FirmwareMeasurements.cpp'),str(directory/'test.cpp'),'-o',str(exe)],check=True)
            data=collector.sanitize(json.loads(subprocess.check_output([str(exe)])))
            self.assertEqual((data['maxGapMs'],data['overruns'],data['encodeUs']),(102,1,8))
            self.assertEqual([e['end'] for e in data['events']],[False,True])
            self.assertEqual(data['events'][-1]['stackBytes'],321)
            self.assertEqual(data['events'][-1]['phase'],'dnsTcpTlsOrReuse')

    def test_collector_resumes_after_unreachable_without_fabricating_samples(self):
        import importlib.util,io,json
        from unittest.mock import patch
        spec=importlib.util.spec_from_file_location('collector',ROOT/'tools/collect_measurements.py')
        collector=importlib.util.module_from_spec(spec);spec.loader.exec_module(collector)
        row=dict.fromkeys(collector.NUMBERS,0);row.update(schemaVersion=1,events=[])
        class Opener:
            calls=0
            def open(self,*_,**__):
                self.calls+=1
                if self.calls==1:raise OSError('private URL must not enter evidence')
                return io.BytesIO(json.dumps(row).encode())
        with tempfile.TemporaryDirectory() as work:
            target=Path(work)/'metrics.jsonl'
            with patch.object(collector.urllib.request,'build_opener',return_value=Opener()),patch.object(collector.time,'sleep'),patch.object(collector.time,'monotonic',side_effect=[0,0.1,0.2,0.3,0.4,0.5,0.6,1.1]):
                result=collector.collect('http://127.0.0.1/measurements',target,1,0.5)
            self.assertEqual(result['samples'],1);self.assertEqual(result['unreachableSamples'],1)
            rows=[json.loads(line) for line in target.read_text().splitlines()]
            self.assertTrue(rows[0]['unreachable']);self.assertNotIn('maxGapMs',rows[0]);self.assertEqual(rows[1]['iterations'],0)
            self.assertEqual(target.stat().st_mode&0o777,0o600)
