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

    def launch(self, package: str, script: str, args: list[str] = [], block=True, **rosparams) -> subprocess.CompletedProcess | subprocess.Popen:
        utils.ensure_installed(["ros2", "cargo", "bash"])

        if self.needs_rebuild():
            self.build()

        setup_file = self.setup_file()
        command = f"source {setup_file} && ros2 launch {package} {script} {' '.join(args)}"

        for key, value in rosparams.items():
            command += f" {key}:={value}"

        print("Launching ROS 2 script:", command)

        if block:
            return subprocess.run(["bash", "-c", command], cwd=self.path)
        else:
            return subprocess.Popen(["bash", "-c", command], cwd=self.path)

    def run(self, package: str, node: str, block=True, ros_args: list[str] = [], **rosparams) -> subprocess.CompletedProcess | subprocess.Popen:
        utils.ensure_installed(["ros2", "cargo", "bash"])

        if self.needs_rebuild():
            self.build()

        ros_args += [f"-p {key}:={str(value).lower()}" for key, value in rosparams.items()]

        setup_file = self.setup_file()
        command = f"source {setup_file} && ros2 run {package} {node} --ros-args {' '.join(ros_args)}"

        print("Running ROS 2 node:", command)

        if block:
            return subprocess.run(["bash", "-c", command], cwd=self.path)
        else:
            return subprocess.Popen(["bash", "-c", command], cwd=self.path)
