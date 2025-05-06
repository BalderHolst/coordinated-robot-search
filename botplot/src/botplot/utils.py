import os
import subprocess

def ensure_installed(programs: str | list[str]) -> bool:
    if isinstance(programs, str): programs = [programs]
    for program in programs:
        try:
            path = subprocess.run(["which", program], capture_output=True, text=True)
            if path.returncode != 0:
                print(f"Error: '{program}' not found.")
                return False
        except Exception as e:
            print(f"Error: {e}")
            return False
    return True

IGNORE = ["target", ".git" ]
def any_file_newer(dir: str, exe_mtime: float) -> bool:
    """Test if any file in the directory is newer than the given mtime."""
    for entry in os.scandir(dir):
        if entry.name in IGNORE: continue
        if entry.is_dir(follow_symlinks=False):
            if any_file_newer(entry.path, exe_mtime): return True
        elif entry.is_file(follow_symlinks=False):
            if os.path.getmtime(entry.path) > exe_mtime:
                return True
