"""Checkout launcher contracts, without starting the web server or devices."""
import contextlib
import importlib.util
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from openarm_platform.model_workspace import child_environment

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('repository_platform_launcher', ROOT/'run_platform.py')
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


class LauncherTests(unittest.TestCase):
    def test_help_needs_neither_environment_nor_configuration(self):
        with patch.object(launcher.os, 'execve') as execute, \
             patch.object(Path, 'is_file', side_effect=AssertionError('help must be early')), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            with self.assertRaises(SystemExit) as caught:
                launcher.main(['--help'])
        self.assertEqual(caught.exception.code, 0)
        self.assertIn('--profile', output.getvalue())
        execute.assert_not_called()

    def test_explicit_external_profile_and_checkout_source_are_forwarded(self):
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory)/'personal.json'
            profile.write_text('{}')
            with patch.object(Path, 'is_file', return_value=True), \
                 patch.object(launcher.os, 'chdir') as chdir, \
                 patch.object(launcher.os, 'execve') as execute, \
                 patch.dict(os.environ, {'PYTHONPATH': '/old-package'}):
                launcher.main(['--profile', str(profile), '--port', '9999', '--attach-only'])
            python = str(ROOT/'.venv/bin/python')
            command, argv, env = execute.call_args.args
            self.assertEqual(command, python)
            self.assertEqual(argv, [python, '-m', 'openarm_runtime', 'platform', '--profile',
                                    str(profile.resolve()), '--port', '9999', '--attach-only'])
            self.assertEqual(env['OPENARM_RUNTIME_SOURCE'], str(ROOT/'src'))
            self.assertEqual(env['PYTHONPATH'].split(os.pathsep), [str(ROOT/'src'), '/old-package'])
            chdir.assert_called_once_with(ROOT)

    def test_missing_environment_returns_actionable_error_without_exec(self):
        with patch.object(Path, 'is_file', return_value=False), \
             patch.object(launcher.os, 'execve') as execute, \
             contextlib.redirect_stderr(io.StringIO()) as output:
            self.assertEqual(launcher.main([]), 1)
        self.assertIn('docs/platform.md', output.getvalue())
        execute.assert_not_called()

    def test_child_keeps_checkout_first_and_model_dependencies(self):
        with patch.dict(os.environ, {'OPENARM_RUNTIME_SOURCE': '/checkout/src',
                                    'PYTHONPATH': '/old-package'}):
            env = child_environment({'PYTHONPATH': '/model-dependencies', 'OMP_NUM_THREADS': '2'})
        self.assertEqual(env['PYTHONPATH'], '/checkout/src'+os.pathsep+'/model-dependencies')
        self.assertEqual(env['OMP_NUM_THREADS'], '2')

    def test_packaged_launch_without_checkout_marker_preserves_model_environment(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(child_environment({'PYTHONPATH': '/deps'})['PYTHONPATH'], '/deps')
