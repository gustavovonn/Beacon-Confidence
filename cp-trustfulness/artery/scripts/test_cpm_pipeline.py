#!/usr/bin/env python3
"""
test_cpm_pipeline.py
Checks for the confidence pipeline.

1. C++ unit tests of the OMNeT-free core (cp-trustfulness/tests/run_tests.sh)
2. Configuration integrity (CPM port, physical sensors only, YAML keys known to the runner)
3. Headless smoke simulation that must actually receive and resolve CPMs (skip with --no-sim)
"""

import os
import sys

# --- VENV AUTO-BOOTSTRAP ---
if sys.prefix == sys.base_prefix:
    venv_python = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".venv", "bin", "python3"))
    if os.path.exists(venv_python):
        os.execv(venv_python, [venv_python] + sys.argv)
# ---------------------------

import subprocess
import unittest
import xml.etree.ElementTree as ET

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import run_cpm_experiment as runner  # noqa: E402

ARTERY_DIR = runner.ARTERY_DIR
CORE_TESTS = os.path.join(runner.WORKSPACE_DIR, "cp-trustfulness", "tests", "run_tests.sh")
CPM_BTP_PORT = "2009"  # vanetza::btp::ports::CPM


class TestCore(unittest.TestCase):
    def test_cpp_unit_tests(self):
        res = subprocess.run(["bash", CORE_TESTS], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.assertEqual(res.returncode, 0, res.stdout[-3000:])


class TestConfiguration(unittest.TestCase):
    def test_cp_services_listen_on_cpm_port(self):
        for name in ("services_cpm.xml", "faulty_services_cpm.xml"):
            root = ET.parse(os.path.join(ARTERY_DIR, "config", name)).getroot()
            cp = [s for s in root.findall("service") if s.get("name") == "CP"]
            self.assertEqual(len(cp), 1, name)
            self.assertEqual(cp[0].find("listener").get("port"), CPM_BTP_PORT, name)

    def test_sensors_are_physical_only(self):
        root = ET.parse(os.path.join(ARTERY_DIR, "config", "sensors.xml")).getroot()
        types = [s.get("type") for s in root.findall("sensor")]
        self.assertTrue(types)
        for t in types:
            self.assertNotIn("CamSensor", t, "CamSensor inserts ground truth of every CAM sender")
            self.assertNotIn("SeeThrough", t, "see-through sensors ignore line of sight")

    def test_yaml_keys_are_known(self):
        with open(os.path.join(ARTERY_DIR, "cpm_batch_config.yaml")) as f:
            cfg = yaml.safe_load(f)
        defaults = cfg["defaults"]
        for block, runs in cfg.items():
            if block == "defaults":
                continue
            for run in runs:
                merged = {**defaults, **run}
                merged.pop("seeds", None)
                runner.omnet_overrides(merged)  # raises on unknown keys
                mix = runner.vehicle_mix(merged)
                self.assertAlmostEqual(sum(mix.values()), 1.0, places=6, msg=f"{block}/{run['id']}")


def run_smoke_simulation() -> bool:
    """10 s grid run with 20 % faulty stations; CPMs must be received and resolved, claims must be scored."""
    print("\n--- Smoke simulation (grid, 10 s) ---")
    cfg = {"id": "smoke", "scenario": "grid", "time_limit": "10s", "faulty_rate": 0.2,
           "gnss_fault_profile": "const_pos_offset", "gnss_fault_start_time": 0.0,
           "cpm_fault_profile": "ghost", "cpm_fault_start_time": 0.0}
    metrics = runner.execute_run(cfg, 1, os.path.join(ARTERY_DIR, "results", "smoke", "pipeline_test"), False)
    if metrics is None:
        print("❌ simulation failed")
        return False
    ok = (metrics["cpms_received"] > 0 and metrics["assoc_precision"] > 0.5 and metrics["invalid_cpms"] == 0
          and metrics["cam_honest_n"] > 0 and metrics["object_honest_n"] > 0)
    print(f"{'✅' if ok else '❌'} cpms={metrics['cpms_received']:.0f} "
          f"assoc_precision={metrics['assoc_precision']:.3f} invalid_cpms={metrics['invalid_cpms']:.0f}")
    return ok


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(TestCore)
    suite.addTests(unittest.TestLoader().loadTestsFromTestCase(TestConfiguration))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():
        sys.exit(1)
    if "--no-sim" not in sys.argv and not run_smoke_simulation():
        sys.exit(1)
