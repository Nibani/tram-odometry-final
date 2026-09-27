"""Independent known-good/bad controls for the authored pure performance gate."""
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
from pathlib import Path

OWN = Path(__file__).resolve().parent
SOURCE = OWN / 'ros_checker_replay.py'
import argparse
parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True);OUT=parser.parse_args().out
OUT.mkdir(parents=True,exist_ok=True)
spec = importlib.util.spec_from_file_location('independently_audited_replay', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

GOOD = {
    'coverage': {'velocity_outputs': 2000, 'position_outputs': 1990},
    'command_to_result_latency_wall_s': {
        'velocity_n': 2000, 'position_n': 1990,
        'velocity': {'p99': .015, 'max': .09},
        'position': {'p99': .020, 'max': .095}},
    'velocity_output_hz_in_header_time': 20.,
    'position_output_hz_in_header_time': 20.,
    'node_resources': {'samples': 300, 'peak_sampled_rss_kib': 45000,
                       'mean_cpu_cores': .15, 'affinity_cores': 2}}

def set_path(report, path, value):
    for key in path[:-1]:
        report = report[key]
    report[path[-1]] = value

def main():
    records = []
    checks = module.performance_checks(deepcopy(GOOD))
    assert len(checks) == 12 and all(item['passed'] for item in checks)
    records.append({'name': 'known_good_all_twelve_checks', 'passed': True})
    boundary = deepcopy(GOOD)
    for kind in ('velocity', 'position'):
        boundary['coverage'][kind+'_outputs'] = 2000
        boundary['command_to_result_latency_wall_s'][kind+'_n'] = 1990
        boundary['command_to_result_latency_wall_s'][kind] = {'p99': .1, 'max': .25}
        boundary[kind+'_output_hz_in_header_time'] = 10.
    boundary['node_resources'] = {'samples': 2, 'peak_sampled_rss_kib': 524288,
                                  'mean_cpu_cores': 2., 'affinity_cores': 2}
    assert all(item['passed'] for item in module.performance_checks(boundary))
    records.append({'name': 'inclusive_documented_thresholds', 'passed': True})
    cases = []
    for kind in ('velocity', 'position'):
        cases.extend([
            (kind+'_insufficient_latency_coverage', ('command_to_result_latency_wall_s',kind+'_n'), 1, kind+'_latency_measurement_coverage'),
            (kind+'_excessive_p99', ('command_to_result_latency_wall_s',kind,'p99'), .100001, kind+'_latency_p99_below_100ms'),
            (kind+'_excessive_max', ('command_to_result_latency_wall_s',kind,'max'), .250001, kind+'_latency_max_below_250ms'),
            (kind+'_too_slow', (kind+'_output_hz_in_header_time',), 9.999, kind+'_publication_at_least_10Hz'),
            (kind+'_no_outputs', ('coverage',kind+'_outputs'), 0, kind+'_latency_measurement_coverage'),
            (kind+'_nan_latency', ('command_to_result_latency_wall_s',kind,'p99'), math.nan, kind+'_latency_p99_below_100ms'),
            (kind+'_negative_latency', ('command_to_result_latency_wall_s',kind,'max'), -.01, kind+'_latency_max_below_250ms'),
            (kind+'_nan_frequency', (kind+'_output_hz_in_header_time',), math.nan, kind+'_publication_at_least_10Hz')])
    cases.extend([
        ('one_resource_sample', ('node_resources','samples'), 1, 'node_resource_measurement_present'),
        ('memory_above_limit', ('node_resources','peak_sampled_rss_kib'), 524289, 'sampled_node_memory_below_512MiB'),
        ('cpu_above_limit', ('node_resources','mean_cpu_cores'), 2.00001, 'node_mean_cpu_below_two_cores'),
        ('unbounded_affinity', ('node_resources','affinity_cores'), 3, 'node_affinity_at_most_two_cores'),
        ('nan_cpu', ('node_resources','mean_cpu_cores'), math.nan, 'node_mean_cpu_below_two_cores'),
        ('missing_resources', ('node_resources',), None, 'node_resource_measurement_present'),
        ('missing_latencies', ('command_to_result_latency_wall_s',), None, 'velocity_latency_measurement_coverage')])
    for name, path, value, target in cases:
        report = deepcopy(GOOD)
        set_path(report, path, value)
        result = {item['name']: item['passed'] for item in module.performance_checks(report)}
        assert result[target] is False, (name, result)
        assert not all(result.values()), name
        records.append({'name': name, 'passed': True, 'expected_failed_assertion': target})
    assert not all(item['passed'] for item in module.performance_checks({}))
    records.append({'name': 'empty_report_rejected', 'passed': True})
    text = SOURCE.read_text()
    assert "report['assertions'].extend(performance_checks(report))\n        if report['status'] != 'ERROR':" in text
    records.append({'name': 'performance_checked_before_final_status_error_preserved_static', 'passed': True})
    out = {'status': 'PASS', 'audited_source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
           'checks_in_good_report': len(checks), 'controls': records}
    (OUT/'performance_gate_controls.json').write_text(json.dumps(out, indent=2), encoding='utf-8')
    print(json.dumps({'status': out['status'], 'controls': len(records), 'checks': len(checks),
                      'audited_source_sha256': out['audited_source_sha256']}, indent=2))

if __name__ == '__main__':
    main()
