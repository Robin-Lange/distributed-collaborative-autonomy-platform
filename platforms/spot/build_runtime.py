"""Build the standalone Spot sidecar for the current release runner."""

from __future__ import annotations

import argparse
import hashlib
import platform
import subprocess
from pathlib import Path


TARGET_FILENAMES = {
    ("linux", "x86_64"): "spot-sidecar-x86_64-unknown-linux-gnu",
    ("linux", "aarch64"): "spot-sidecar-aarch64-unknown-linux-gnu",
    ("darwin", "aarch64"): "spot-sidecar-aarch64-apple-darwin",
    ("windows", "x86_64"): "spot-sidecar-x86_64-pc-windows-msvc.exe",
}
SDK_RESOURCE_PACKAGE = "bosdyn.client.resources"


def normalized_host() -> tuple[str, str]:
    operating_system = platform.system().lower()
    architecture = platform.machine().lower()
    if architecture in {"amd64", "x64"}:
        architecture = "x86_64"
    elif architecture == "arm64":
        architecture = "aarch64"
    return operating_system, architecture


def artifact_filename(host: tuple[str, str]) -> str:
    try:
        return TARGET_FILENAMES[host]
    except KeyError as error:
        operating_system, architecture = host
        raise RuntimeError(
            f"Spot runtime is not published for {operating_system}/{architecture}"
        ) from error


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as artifact:
        for chunk in iter(lambda: artifact.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def pyinstaller_arguments(
    output_directory: Path,
    work_directory: Path,
    spot_directory: Path,
    executable_name: str,
) -> list[str]:
    return [
        "--clean",
        "--noconfirm",
        "--onefile",
        "--name",
        executable_name,
        "--distpath",
        str(output_directory),
        "--workpath",
        str(work_directory / "build"),
        "--specpath",
        str(work_directory),
        "--paths",
        str(spot_directory),
        "--collect-all",
        SDK_RESOURCE_PACKAGE,
        str(spot_directory / "spot_sidecar.py"),
    ]


def verify_runtime(artifact: Path) -> None:
    subprocess.run([str(artifact), "--runtime-self-test"], check=True)


def build(output_directory: Path) -> Path:
    import PyInstaller.__main__

    spot_directory = Path(__file__).resolve().parent
    output_directory.mkdir(parents=True, exist_ok=True)
    filename = artifact_filename(normalized_host())
    executable_name = filename.removesuffix(".exe")
    work_directory = output_directory.parent / "pyinstaller-work"

    PyInstaller.__main__.run(
        pyinstaller_arguments(
            output_directory,
            work_directory,
            spot_directory,
            executable_name,
        )
    )

    artifact = output_directory / filename
    if not artifact.is_file():
        raise RuntimeError(f"PyInstaller did not produce {artifact}")

    verify_runtime(artifact)
    checksum = sha256(artifact)
    artifact.with_name(f"{filename}.sha256").write_text(
        f"{checksum}  {filename}\n", encoding="ascii"
    )
    return artifact


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("dist/spot"),
        help="directory for the executable and SHA-256 file",
    )
    args = parser.parse_args()
    artifact = build(args.output.resolve())
    print(f"built {artifact}")


if __name__ == "__main__":
    main()
