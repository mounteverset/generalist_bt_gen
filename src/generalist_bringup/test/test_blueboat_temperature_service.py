import importlib.util
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'blueboat_temperature_service.py'
)
SPEC = importlib.util.spec_from_file_location('blueboat_temperature_service', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_temperature_format_rejects_invalid_samples():
    assert MODULE.format_temperature(21.2346) == '21.235 °C'
    try:
        MODULE.format_temperature(float('nan'))
    except ValueError:
        pass
    else:
        raise AssertionError('non-finite temperature was accepted')
