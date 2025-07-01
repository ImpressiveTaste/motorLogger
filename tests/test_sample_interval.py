import importlib
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

# Provide a dummy serial module so MotorLogger can be imported without pyserial
dummy_serial = types.ModuleType("serial")
dummy_tools = types.ModuleType("serial.tools")
dummy_tools.list_ports = types.ModuleType("serial.tools.list_ports")
dummy_tools.list_ports.comports = lambda: []
sys.modules.setdefault("serial", dummy_serial)
sys.modules.setdefault("serial.tools", dummy_tools)
sys.modules.setdefault("serial.tools.list_ports", dummy_tools.list_ports)

# Stub pyx2cscope module expected by MotorLogger when USE_SCOPE is True
dummy_scope_mod = types.ModuleType("pyx2cscope.x2cscope")
dummy_scope_mod.X2CScope = object
sys.modules.setdefault("pyx2cscope.x2cscope", dummy_scope_mod)

import MotorLogger as ml
ml.USE_SCOPE = False


def test_min_interval_default(monkeypatch):
    monkeypatch.setattr(ml, 'ENFORCE_SAMPLE_LIMIT', True)
    assert ml.min_allowed_interval(1) == ml.MIN_DELAY_PER_VAR_MS
    assert ml.min_allowed_interval(3) == 3 * ml.MIN_DELAY_PER_VAR_MS


def test_min_interval_disabled(monkeypatch):
    monkeypatch.setattr(ml, 'ENFORCE_SAMPLE_LIMIT', False)
    assert ml.min_allowed_interval(5) == 0.0
