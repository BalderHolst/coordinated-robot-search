import subprocess
import os
from typing import Self

class RustCrate:
    name: str
    path: str
    dependencies: list[Self] = []

    def __init__(self, path: str, name: str = None, dependencies: list[Self] = []):
        if name is None:
            name = os.path.basename(path)

        self.path = path
        self.name = name
        self.dependencies = dependencies

    def compile(self, flags: list[str] = []):
        """Compile the Rust crate."""
        print(f"Compiling {self.path}...")
        proc = subprocess.run(["cargo", "build", "--release"] + flags, cwd=self.path)
        if proc.returncode != 0:
            print(f"Error compiling '{self.name}':\n{proc.stderr}")
            exit(1)

    def executable(self) -> str:
        return os.path.join(self.path, "target", "release", self.name)

    def needs_recompile(self, exe_path = None) -> bool:
        """Check if any source files are newer than the build artifact."""

        if exe_path is None:
            exe_path = self.executable()

        # Check if the executable exists
        if not os.path.exists(exe_path):
            return True

        exe_mtime = os.path.getmtime(exe_path)

        # Check if dependencies need recompilation
        for dep in self.dependencies:
            if dep.needs_recompile(exe_path=exe_path):
                return True

        # Walk source files and check modification times
        for root, _, files in os.walk(self.path):
            for file in files:
                if file.endswith(".rs") or file in ("Cargo.toml", "Cargo.lock"):
                    full_path = os.path.join(root, file)
                    if os.path.getmtime(full_path) > exe_mtime:
                        return True

        return False

    def run(self, flags: list[str] = [], **kwargs) -> subprocess.CompletedProcess:
        """Run the Rust crate with the given flags."""
        if self.needs_recompile(): self.compile()
        proc = subprocess.run([self.executable()] + flags, **kwargs)
        if proc.returncode != 0:
            print(f"Error: {proc.stderr}")
            exit(1)
        return proc
