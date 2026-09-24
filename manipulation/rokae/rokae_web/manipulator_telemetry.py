"""Three read-only cache workers sharing the stable SDK owner and motion lock."""
import copy
from datetime import datetime, timezone
import math
import threading
import time

from rokae_robot_state import COMPONENTS, ManipulatorStateSnapshot
from .telemetry import TelemetryBusy


class ManipulatorTelemetry:
    def __init__(self, reader, config, legacy_config=None):
        self.reader = reader
        hz = float(config.get('sample_hz',10))
        self.freshness_ms = float(config.get('freshness_ms',500))
        if not math.isfinite(hz) or not .1 <= hz <= 10 or not math.isfinite(self.freshness_ms) or self.freshness_ms < 2000/hz:
            raise ValueError('invalid manipulator sample_hz/freshness_ms')
        for key, minimum in (('log_query_interval_sec',5),('gripper_query_interval_sec',1)):
            value = float(config.get(key,minimum))
            if not math.isfinite(value) or value < minimum:raise ValueError('invalid '+key)
        self.interval = 1/hz
        self.legacy_stale = float((legacy_config or {}).get('stale_after_seconds',3))
        self._lock, self._stop = threading.Lock(), threading.Event()
        self._threads = []
        self._records = {n:dict(sample_count=0,error_count=0,connected=False,last_error_code='not_sampled') for n in COMPONENTS}

    def start(self):
        with self._lock:
            if self._threads:return
            for name in COMPONENTS:
                thread = threading.Thread(target=self._run,args=(name,),name='rokae-state-'+name,daemon=True)
                self._threads.append(thread)
                thread.start()

    def close(self):
        self._stop.set()
        for thread in self._threads:thread.join(timeout=2)
        # The hardware service, not the sampler, owns SDK connection shutdown.

    def _run(self, name):
        self._stop.wait(COMPONENTS.index(name)*self.interval/3)
        while not self._stop.is_set():
            started = time.monotonic()
            self.poll_once(name)
            self._stop.wait(max(.01,self.interval-(time.monotonic()-started)))

    def poll_once(self, name):
        started, measured = time.monotonic(), int(time.time()*1000)
        try:
            data = self.reader.sample(name)
            for part in data.values():part['measured_at_ms'] = measured
        except TelemetryBusy:
            return  # Motion contention is neither connection failure nor a new sample.
        except Exception as exc:
            with self._lock:
                record = self._records[name]
                record.update(connected=False,last_error_code=(type(exc).__name__+': '+str(exc))[:160])
                record['error_count'] += 1
            return
        with self._lock:
            record = self._records[name]
            record.update(connected=True,last_error_code=None,monotonic=started,
                          measured_at_ms=measured,data=copy.deepcopy(data),
                          read_duration_ms=(time.monotonic()-started)*1000)
            record['sample_count'] += 1

    def get_state(self):
        with self._lock:records = copy.deepcopy(self._records)
        now, data = time.monotonic(), {}
        health = {}
        for name, record in records.items():
            age = max(0,(now-record['monotonic'])*1000) if 'monotonic' in record else None
            fresh = record['connected'] and age is not None and age <= self.freshness_ms
            health[name] = {k:record[k] for k in ('connected','sample_count','error_count','last_error_code')}
            health[name].update(fresh=fresh,last_sample_age_ms=age)
            if fresh:data.update(record['data'])
        timestamps = [r['measured_at_ms'] for r in records.values() if 'measured_at_ms' in r]
        data.update(schema_version=1, measured_at_ms=max(timestamps) if timestamps else int(time.time()*1000),
                    dual_arm_available=all(n in data for n in COMPONENTS[:2]),health=health)
        return ManipulatorStateSnapshot(data)

    def snapshot(self, module=None):
        """Preserve the 8092 upper-body HTTP contract using the same sample cache."""
        names = {'left_arm':'left_arm','right_arm':'right_arm','trunk':'body'}
        if module is not None and module not in names:raise ValueError('unknown module')
        with self._lock:records = copy.deepcopy(self._records)
        result, now = {}, time.monotonic()
        for legacy, name in names.items():
            if module is not None and legacy != module:continue
            record = records[name]
            age = max(0,now-record['monotonic']) if 'monotonic' in record else None
            fresh = record['connected'] and age is not None and age <= self.legacy_stale
            part = record.get('data',{}).get(name,{})
            values = dict(state=None,joint_positions_deg=None,end_pose=None)
            if fresh:
                values = dict(state=part['operation_state'],joint_positions_deg=part['joint_positions_deg'],
                              end_pose=dict(position_mm=part['end_pose'][:3],rpy_deg=part['end_pose'][3:],
                                            frame=legacy+'_controller_base',coordinate_type='flangeInBase'))
            measured = record.get('measured_at_ms')
            result[legacy] = dict(**values,valid=fresh,stale=not fresh,
                sampled_at=datetime.fromtimestamp(measured/1000,timezone.utc).isoformat() if measured else None,
                age_ms=age*1000 if age is not None else None,sequence=record['sample_count'],
                read_duration_ms=record.get('read_duration_ms'),
                error=record['last_error_code'] or (None if fresh else '采样已过期'))
        return dict(schema_version=1,mode='hardware',poll_interval_seconds=self.interval,
                    stale_after_seconds=self.legacy_stale,all_valid=all(p['valid'] for p in result.values()),modules=result)
