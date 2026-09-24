"""Explicit SDK value protocol. No pickle, native robot objects, or code execution.

Web planners use lightweight value objects. Only the hardware service imports
the native SDK and reconstructs its value types immediately before a call.
"""
from __future__ import annotations

import copy
from types import SimpleNamespace


FIELDS = {
    "Frame": ("trans", "rpy"),
    "CartesianPosition": ("trans", "rpy", "elbow", "hasElbow", "confData", "external"),
    "JointPosition": ("joints", "external"),
    "Load": ("mass", "cog", "inertia"),
    "Toolset": ("load", "end", "ref"),
    "MoveLCommand": ("target", "speed", "zone", "rotSpeed", "customInfo"),
    "MoveJCommand": ("target", "speed", "zone", "jointSpeed", "customInfo"),
    "MoveAbsJCommand": ("target", "speed", "zone", "jointSpeed", "customInfo"),
}
HOLDERS = {"PyString": "", "PyTypeBool": False, "PyTypeDouble": 0.0,
           "PyTypeVectorArrayDouble2": [], "PyTypeVectorInt": [], "PyTypeVectorString": [], "PyTypeVectorBool": []}
ENUMS = {"CoordinateType", "OperateMode", "MotionControlMode", "DragParameterSpace",
         "DragParameterType", "OperationState", "PowerState", "xPanelOptVout"}


class EnumValue:
    def __init__(self, kind, name):
        self.kind, self.name = kind, name

    def __str__(self):
        return self.name


class EnumValues:
    def __init__(self, kind):
        self.kind = kind

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        return EnumValue(self.kind, name)


class Value:
    def __init__(self, *args):
        name = type(self).__name__
        if name in ("Frame", "CartesianPosition"):
            pose = list(args[0]) if args else [0.0]*6
            if len(pose) != 6:
                raise ValueError("SDK broker pose requires six m/rad values")
            self.trans, self.rpy = pose[:3], pose[3:]
            if name == "CartesianPosition":
                self.elbow, self.hasElbow, self.confData, self.external = 0.0, False, [], []
        elif name == "JointPosition":
            self.joints, self.external = list(args[0]) if args else [], []
        elif name.startswith("Move"):
            self.target = args[0]
            self.speed = float(args[1]) if len(args) > 1 else -1.0
            self.zone = float(args[2]) if len(args) > 2 else -1.0
            self.customInfo = ""
            setattr(self, "rotSpeed" if name == "MoveLCommand" else "jointSpeed", -1.0)
        elif name == "Load":
            self.mass, self.cog, self.inertia = 0.0, [0.0]*3, [0.0]*3
        elif name == "Toolset":
            self.load, self.end, self.ref = SDK.Load(), SDK.Frame(), SDK.Frame()


class Holder:
    def __init__(self, value=None):
        self._value = copy.deepcopy(HOLDERS[type(self).__name__] if value is None else value)

    def content(self):
        return copy.deepcopy(self._value)

    get = content


SDK = SimpleNamespace(**{n: type(n, (Value,), {}) for n in FIELDS},
                      **{n: type(n, (Holder,), {}) for n in HOLDERS},
                      **{n: EnumValues(n) for n in ENUMS})


def encode(value):
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [encode(v) for v in value]
    if isinstance(value, dict):
        return {str(k): encode(v) for k, v in value.items()}
    name = type(value).__name__
    if isinstance(value, EnumValue):
        return {"$enum": value.kind, "name": value.name}
    if name in ENUMS:
        return {"$enum": name, "name": value.name}
    if name in HOLDERS:
        return {"$holder": name, "value": encode(value.content())}
    if name in FIELDS:
        return {"$type": name, "fields": {f: encode(getattr(value, f)) for f in FIELDS[name]}}
    raise TypeError(f"Unsupported SDK wire value: {name}")


def decode(value, sdk=SDK):
    if isinstance(value, list):
        return [decode(v, sdk) for v in value]
    if not isinstance(value, dict):
        return value
    if "$enum" in value:
        if value["$enum"] not in ENUMS:
            raise ValueError("Unsupported SDK enum")
        return getattr(getattr(sdk, value["$enum"]), value["name"])
    if "$holder" in value:
        if value["$holder"] not in HOLDERS:
            raise ValueError("Unsupported SDK holder")
        return getattr(sdk, value["$holder"])(decode(value["value"], sdk))
    if "$type" in value:
        name = value["$type"]
        if name not in FIELDS or set(value["fields"]) != set(FIELDS[name]):
            raise ValueError("SDK wire fields do not match the supported schema")
        f = {k: decode(v, sdk) for k, v in value["fields"].items()}
        cls = getattr(sdk, name)
        if name in ("Frame", "CartesianPosition"):
            obj = cls(f["trans"] + f["rpy"])
        elif name == "JointPosition":
            obj = cls(f["joints"])
        elif name.startswith("Move"):
            obj = cls(f["target"], f["speed"], f["zone"])
        else:
            obj = cls()
        for key, item in f.items():
            setattr(obj, key, item)
        return obj
    return {k: decode(v, sdk) for k, v in value.items()}


def apply_outputs(original, returned):
    """Copy SDK out-parameters back into the web caller's original holders."""
    if isinstance(original, dict) and isinstance(returned, dict):
        original.clear(); original.update(returned)
    elif isinstance(original, list) and isinstance(returned, list):
        original[:] = returned
    elif isinstance(original, Holder) and isinstance(returned, Holder):
        original._value = returned.content()
    elif isinstance(original, Value) and type(original) is type(returned):
        for name in FIELDS[type(original).__name__]:
            setattr(original, name, getattr(returned, name))
