import os
import sys
import tempfile
import unittest
from unittest import mock


SUPERVISOR_DIR = os.path.join(
    os.path.dirname(__file__), '..', 'controllers', 'factory_supervisor')
sys.path.insert(0, os.path.abspath(SUPERVISOR_DIR))

from factory_supervisor import FactorySupervisor


class FakeAnimationSupervisor:
    def __init__(self, start_ok=True, stop_ok=True):
        self.start_ok = start_ok
        self.stop_ok = stop_ok
        self.started = []
        self.stop_calls = 0

    def animationStartRecording(self, path):
        self.started.append(path)
        return self.start_ok

    def animationStopRecording(self):
        self.stop_calls += 1
        return self.stop_ok


class WebotsRecordingTests(unittest.TestCase):
    def _factory(self, fake):
        factory = FactorySupervisor.__new__(FactorySupervisor)
        factory.supervisor = fake
        return factory

    @staticmethod
    def _write_animation_files(html_path):
        stem, _extension = os.path.splitext(html_path)
        for path in (html_path, stem + '.json', stem + '.x3d'):
            with open(path, 'wb') as output:
                output.write(b'animation data')

    def test_recording_is_disabled_without_explicit_path(self):
        factory = self._factory(FakeAnimationSupervisor())
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertFalse(factory._start_animation_recording())
        self.assertEqual([], factory.supervisor.started)

    def test_start_accepts_html_path_and_creates_parent(self):
        fake = FakeAnimationSupervisor()
        factory = self._factory(fake)
        with tempfile.TemporaryDirectory() as temp_dir:
            animation = os.path.join(temp_dir, 'nested', 'validation.html')
            with mock.patch.dict(
                    os.environ,
                    {'SMART_FACTORY_ANIMATION_PATH': animation}, clear=True):
                self.assertTrue(factory._start_animation_recording())
            self.assertTrue(os.path.isdir(os.path.dirname(animation)))
            self.assertEqual([os.path.abspath(animation)], fake.started)

    def test_start_rejects_non_html_extension(self):
        fake = FakeAnimationSupervisor()
        factory = self._factory(fake)
        with mock.patch.dict(
                os.environ,
                {'SMART_FACTORY_ANIMATION_PATH': 'validation.mp4'}, clear=True):
            with self.assertRaisesRegex(ValueError, 'html extension'):
                factory._start_animation_recording()
        self.assertEqual([], fake.started)

    def test_start_refuses_to_overwrite_companion_file(self):
        fake = FakeAnimationSupervisor()
        factory = self._factory(fake)
        with tempfile.TemporaryDirectory() as temp_dir:
            animation = os.path.join(temp_dir, 'validation.html')
            with open(os.path.splitext(animation)[0] + '.json', 'wb') as output:
                output.write(b'existing evidence')
            with mock.patch.dict(
                    os.environ,
                    {'SMART_FACTORY_ANIMATION_PATH': animation}, clear=True):
                with self.assertRaisesRegex(FileExistsError, 'overwrite'):
                    factory._start_animation_recording()
        self.assertEqual([], fake.started)

    def test_start_propagates_webots_rejection(self):
        factory = self._factory(FakeAnimationSupervisor(start_ok=False))
        with tempfile.TemporaryDirectory() as temp_dir:
            animation = os.path.join(temp_dir, 'validation.html')
            with mock.patch.dict(
                    os.environ,
                    {'SMART_FACTORY_ANIMATION_PATH': animation}, clear=True):
                with self.assertRaisesRegex(RuntimeError, 'rejected'):
                    factory._start_animation_recording()

    def test_finish_requires_successful_stop_and_all_outputs(self):
        fake = FakeAnimationSupervisor()
        factory = self._factory(fake)
        with tempfile.TemporaryDirectory() as temp_dir:
            animation = os.path.join(temp_dir, 'validation.html')
            self._write_animation_files(animation)
            factory._animation_recording_started = True
            factory._animation_recording_path = animation
            self.assertTrue(factory._finish_animation_recording())
        self.assertEqual(1, fake.stop_calls)
        self.assertFalse(factory._animation_recording_started)

    def test_finish_rejects_missing_companion(self):
        fake = FakeAnimationSupervisor()
        factory = self._factory(fake)
        with tempfile.TemporaryDirectory() as temp_dir:
            animation = os.path.join(temp_dir, 'validation.html')
            with open(animation, 'wb') as output:
                output.write(b'html only')
            factory._animation_recording_started = True
            factory._animation_recording_path = animation
            self.assertFalse(factory._finish_animation_recording())

    def test_finish_propagates_webots_export_failure(self):
        fake = FakeAnimationSupervisor(stop_ok=False)
        factory = self._factory(fake)
        factory._animation_recording_started = True
        factory._animation_recording_path = 'validation.html'
        self.assertFalse(factory._finish_animation_recording())


if __name__ == '__main__':
    unittest.main()
