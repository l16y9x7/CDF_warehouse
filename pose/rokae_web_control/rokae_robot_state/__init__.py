"""Device-side cached state port. The SDK remains owned by mui-sdk.service."""
import copy
import json
import math
import threading
import time
import urllib.request


COMPONENTS = ('left_arm','right_arm','body')


class ManipulatorStateSnapshot:
    def __init__(self, mapping):
        self._mapping = copy.deepcopy(mapping)

    def to_mapping(self):
        return copy.deepcopy(self._mapping)

    def to_legacy_dual_arm_mapping(self):
        if not self._mapping.get('dual_arm_available'):
            return {}
        keys = ('state','joint_positions_deg','end_pose')
        if any(not all(k in self._mapping.get(arm,{}) for k in keys) for arm in COMPONENTS[:2]):
            return {}
        return {arm:{k:copy.deepcopy(self._mapping[arm][k]) for k in keys} for arm in COMPONENTS[:2]}


def empty_mapping(error='not_started'):
    return dict(schema_version=1, measured_at_ms=int(time.time()*1000), dual_arm_available=False,
                health={name:dict(connected=False,fresh=False,sample_count=0,error_count=0,
                                  last_sample_age_ms=None,last_error_code=error) for name in COMPONENTS})


class ManipulatorStatePort:
    """start/stop own only the HTTP cache worker; get_state never performs I/O."""
    def __init__(self, url, sample_hz=10, freshness_ms=500):
        if not math.isfinite(sample_hz) or not 0 < sample_hz <= 10 or not math.isfinite(freshness_ms) or freshness_ms < 200:
            raise ValueError('invalid state cache interval/freshness')
        self.url, self.interval, self.freshness_ms = url, 1/sample_hz, freshness_ms
        self._lock, self._stop = threading.Lock(), threading.Event()
        self._thread = None
        self._mapping, self._received = empty_mapping(), time.monotonic()

    def start(self):
        with self._lock:
            if self._thread is not None and self._thread.is_alive():return
            self._stop.clear()
            self._thread = threading.Thread(target=self._run,name='rokae-state-cache',daemon=True)
            self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:self._thread.join(timeout=3)
        with self._lock:
            self._mapping, self._received = empty_mapping('stopped'), time.monotonic()

    def _poll(self):
        started = time.monotonic()
        try:
            with urllib.request.urlopen(self.url,timeout=2) as response:
                envelope = json.load(response)
            data = envelope['data']
            if data.get('schema_version') != 1 or not all(n in data.get('health',{}) for n in COMPONENTS):
                raise ValueError('invalid manipulator state schema')
            # Include transport time conservatively; cache freshness never resets
            # simply because the same controller sample was fetched again.
            elapsed = (time.monotonic()-started)*1000
            for health in data['health'].values():
                if health.get('last_sample_age_ms') is not None:health['last_sample_age_ms'] += elapsed
            with self._lock:
                self._mapping, self._received = copy.deepcopy(data), time.monotonic()
        except Exception as exc:
            with self._lock:
                elapsed = max(0,(time.monotonic()-self._received)*1000)
                for name in COMPONENTS:
                    health = self._mapping['health'][name]
                    age = health.get('last_sample_age_ms')
                    health.update(connected=False,fresh=False,last_error_code='transport:'+type(exc).__name__,
                                  last_sample_age_ms=age+elapsed if age is not None else None)
                    self._mapping.pop(name,None)
                self._mapping.pop('head',None)
                self._mapping['dual_arm_available'] = False
                self._received = time.monotonic()

    def _run(self):
        while not self._stop.is_set():
            started = time.monotonic()
            self._poll()
            self._stop.wait(max(.01,self.interval-(time.monotonic()-started)))

    def get_state(self):
        with self._lock:
            data, received = copy.deepcopy(self._mapping), self._received
        elapsed = max(0,(time.monotonic()-received)*1000)
        for name in COMPONENTS:
            health = data['health'][name]
            age = health.get('last_sample_age_ms')
            if age is not None:age += elapsed
            health['last_sample_age_ms'] = age
            health['fresh'] = bool(health.get('fresh') and age is not None and age <= self.freshness_ms)
            if not health['fresh']:
                data.pop(name,None)
                if name == 'body':data.pop('head',None)
        data['dual_arm_available'] = all(name in data and data['health'][name]['fresh'] for name in COMPONENTS[:2])
        return ManipulatorStateSnapshot(data)


def build_manipulator_state_service(config):
    settings = config.get('manipulator_state',config.get('robot',{}).get('manipulator_state',{}))
    port = config.get('hardware_service',{}).get('port',8092)
    url = settings.get('url',f'http://127.0.0.1:{port}/api/telemetry/manipulator-state')
    return ManipulatorStatePort(url,float(settings.get('sample_hz',10)),float(settings.get('freshness_ms',500)))
