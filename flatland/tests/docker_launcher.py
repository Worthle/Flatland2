#!/usr/bin/env python3
"""Check host-side Docker selection without requiring a GPU or Docker daemon."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[2]
FAKE_DOCKER = '''import json, os, sys
args = sys.argv[1:]
with open(os.environ['DOCKER_TEST_CALLS'], 'a') as stream:
    stream.write(json.dumps(args) + '\\n')
if args[0] == 'info':
    if os.environ.get('DOCKER_TEST_DAEMON') == 'missing':
        print('Docker daemon unavailable', file=sys.stderr)
        sys.exit(7)
    print(os.environ.get('DOCKER_TEST_RUNTIME', 'nvidia'))
elif args[0] == 'image':
    sys.exit(int(os.environ.get('DOCKER_TEST_IMAGE_MISSING', '0')))
elif args[0] == 'run':
    print(os.environ.get('DOCKER_TEST_DEVICES', 'GPU 0: Test GPU (UUID: GPU-test)'))
    sys.exit(int(os.environ.get('DOCKER_TEST_PROBE_EXIT', '0')))
elif args[0] == 'compose':
    print(json.dumps(dict(args=args, runtime=os.environ['FLATLAND_DOCKER_RUNTIME'],
                         renderer=os.environ['FLATLAND_RENDERER'],
                         devices=os.environ['FLATLAND_NVIDIA_DEVICES'])))
    sys.exit(int(os.environ.get('DOCKER_TEST_COMPOSE_EXIT', '0')))
else:
    sys.exit(99)
'''


class DockerLauncherTest(unittest.TestCase):
    def run_launcher(self, **settings):
        with tempfile.TemporaryDirectory(prefix='.docker-test-', dir=REPO) as tmp:
            directory = Path(tmp)
            docker = directory / 'docker'
            docker.write_text(f'#!{sys.executable}\n' + FAKE_DOCKER)
            docker.chmod(0o755)
            calls = directory / 'calls.jsonl'
            env = {key: value for key, value in os.environ.items()
                   if not key.startswith(('DOCKER_TEST_', 'FLATLAND_'))}
            env.update(PATH=f'{directory}:{env["PATH"]}', DOCKER_TEST_CALLS=str(calls))
            env.update(settings)
            args = ['run', '--rm', 'flatland', 'printf', '%s', 'a path with spaces', '$(literal)']
            result = subprocess.run([str(REPO / 'docker.sh'), *args], env=env,
                                    cwd=directory, text=True, capture_output=True)
            history = [json.loads(line) for line in calls.read_text().splitlines()]
        if result.returncode != 7:
            config = json.loads(result.stdout)
            self.assertEqual(config['args'], ['compose', '--project-directory', str(REPO),
                             '-f', str(REPO / 'docker-compose.yaml'), *args])
        else:
            config = None
        return result, config, history

    def assert_cpu(self, config):
        self.assertEqual((config['runtime'], config['renderer'], config['devices']),
                         ('runc', 'cpu', 'void'))

    def test_nvidia_is_preferred(self):
        result, config, _ = self.run_launcher()
        self.assertEqual(result.returncode, 0)
        self.assertEqual((config['runtime'], config['renderer'], config['devices']),
                         ('nvidia', 'nvidia', 'all'))

    def test_missing_runtime_skips_probe(self):
        result, config, history = self.run_launcher(DOCKER_TEST_RUNTIME='')
        self.assertEqual(result.returncode, 0)
        self.assert_cpu(config)
        self.assertEqual([call[0] for call in history], ['info', 'compose'])

    def test_broken_nvidia_runtime_falls_back(self):
        result, config, _ = self.run_launcher(DOCKER_TEST_PROBE_EXIT='1')
        self.assertEqual(result.returncode, 0)
        self.assert_cpu(config)

    def test_no_gpu_falls_back(self):
        result, config, _ = self.run_launcher(DOCKER_TEST_DEVICES='No devices were found')
        self.assertEqual(result.returncode, 0)
        self.assert_cpu(config)

    def test_first_build_uses_base_image_for_probe(self):
        result, config, history = self.run_launcher(DOCKER_TEST_IMAGE_MISSING='1')
        self.assertEqual(result.returncode, 0)
        self.assertEqual(config['renderer'], 'nvidia')
        probe = next(call for call in history if call[0] == 'run')
        self.assertEqual(probe[-2:], ['ros:humble', '-L'])

    def test_daemon_failure_is_not_hidden(self):
        result, _, history = self.run_launcher(DOCKER_TEST_DAEMON='missing')
        self.assertEqual(result.returncode, 7)
        self.assertIn('Docker daemon unavailable', result.stderr)
        self.assertEqual([call[0] for call in history], ['info'])

    def test_compose_exit_code_is_preserved(self):
        result, _, _ = self.run_launcher(DOCKER_TEST_COMPOSE_EXIT='13')
        self.assertEqual(result.returncode, 13)


if __name__ == '__main__':
    unittest.main()
