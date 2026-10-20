import importlib.util
import pathlib
import subprocess
import unittest
from unittest.mock import patch


ROOT = pathlib.Path(__file__).resolve().parents[2]
BUILD_SCRIPT = ROOT / "platforms" / "spot" / "build_runtime.py"


def load_build_script():
    spec = importlib.util.spec_from_file_location("spot_runtime_build", BUILD_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SpotRuntimeBuildTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build_script = load_build_script()

    def test_release_matrix_has_stable_artifact_names(self):
        expected = {
            ("linux", "x86_64"): "spot-sidecar-x86_64-unknown-linux-gnu",
            ("linux", "aarch64"): "spot-sidecar-aarch64-unknown-linux-gnu",
            ("darwin", "aarch64"): "spot-sidecar-aarch64-apple-darwin",
            ("windows", "x86_64"): "spot-sidecar-x86_64-pc-windows-msvc.exe",
        }
        for host, filename in expected.items():
            with self.subTest(host=host):
                self.assertEqual(
                    self.build_script.artifact_filename(host), filename
                )

    def test_unknown_host_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "not published"):
            self.build_script.artifact_filename(("freebsd", "x86_64"))

    def test_pyinstaller_collects_spot_sdk_certificate_resources(self):
        arguments = self.build_script.pyinstaller_arguments(
            pathlib.Path("dist/spot"),
            pathlib.Path("dist/pyinstaller-work"),
            pathlib.Path("platforms/spot"),
            "spot-sidecar-test",
        )

        collect_index = arguments.index("--collect-all")
        self.assertEqual(
            arguments[collect_index + 1],
            "bosdyn.client.resources",
        )

    def test_runtime_verification_initializes_the_packaged_sdk(self):
        artifact = pathlib.Path("dist/spot/spot-sidecar-test")

        with patch.object(subprocess, "run") as run:
            self.build_script.verify_runtime(artifact)

        run.assert_called_once_with(
            [str(artifact), "--runtime-self-test"],
            check=True,
        )


if __name__ == "__main__":
    unittest.main()
