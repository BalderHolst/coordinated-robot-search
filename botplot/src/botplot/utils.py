import os
import subprocess

def check_installed(programs: str | list[str]) -> bool:
    if isinstance(programs, str): programs = [programs]
    for program in programs:
        try:
            path = subprocess.run(["which", program], capture_output=True, text=True)
            if path.returncode != 0:
                return False
        except Exception as e:
            print(f"Error: {e}")
            return False
    return True

def ensure_installed(programs: str | list[str]) -> bool:
    if isinstance(programs, str): programs = [programs]
    error = False
    for program in programs:
        if not check_installed(program):
            print(f"Error: '{program}' not found.")
            error = True
    if error:
        exit(1)


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

def kill_gazebo():
    """Kill all Gazebo processes."""
    ensure_installed(["bash", "ps", "grep", "awk", "xargs"])
    cmd = "ps aux | grep -E 'rviz|gz|ros' | grep -v -E 'home|grep|python' | awk '{print $2}' | xargs kill"
    subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)
