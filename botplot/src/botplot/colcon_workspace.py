import os
import subprocess

import botplot.utils as utils

class ColconWorkspace:
    def __init__(self, path: str):
        self.path = os.path.abspath(path)

    def src_path(self) -> str:
        """Return the source path for this workspace."""
        return os.path.join(self.path, "src")

    def setup_file(self) -> str:
        """Return the setup file for this workspace."""
        return f"{self.path}/install/setup.bash"

    def timestamp_file(self) -> str:
        """Return the timestamp file for this workspace."""
        return os.path.join(self.path, "build", ".timestamp")

    def needs_rebuild(self) -> bool:
        if not os.path.exists(self.timestamp_file()): return True
        mtime = os.path.getmtime(self.timestamp_file())
        return utils.any_file_newer(self.src_path(), mtime)

    def build(self):
        utils.ensure_installed("colcon")
        print(f"Building colcon workspace at {self.path}...")
        subprocess.run(["colcon", "build"], cwd=self.path)

        with open(os.path.join(self.path, "build", ".timestamp"), "w") as f:
            f.write("")

    def launch(self, package: str, script: str, args: list[str] = [], **rosargs):
        utils.ensure_installed("ros2")

        setup_file = self.setup_file()

        if self.needs_rebuild():
            self.build()

        command = f"source {setup_file} && ros2 launch {package} {script} {' '.join(args)}"

        for key, value in rosargs.items():
            command += f" {key}:={value}"

        subprocess.run(["bash", "-c", command], cwd=self.path)

