import os
import subprocess
from dataclasses import dataclass

@dataclass
class Robot:
    x: float = 0.0
    y: float = 0.0
    angle: float = 0.0

@dataclass
class Scenario:
    world: str
    behavior: str
    duration: int
    robots: list[Robot]

    def to_ron(self) -> str:
        robots = ", ".join([f"(pos: (x: {r.x}, y: {r.y}), angle: {r.angle})" for r in self.robots])

        return f"""
        Scenario(
            world: "{self.world}",
            behavior: "{self.behavior}",
            duration: {self.duration},
            robots: [{robots}]
        )
        """

def run_sim(scenario: Scenario | str, output: str, headless=True):

    # Get simulator binary from $SIMULATOR environment variable
    simulator = os.environ.get("SIMULATOR")

    # Check if simulator binary exists
    if not simulator:
        print("Error: $SIMULATOR environment variable is not set. Please set it to the simulator binary path.")
        exit(1)

    # Check that the simulator path is valid
    if not os.path.isfile(simulator):
        print("Error: $SIMULATOR environment variable is not set to a file path.")
        exit(1)

    flags = []
    if headless:
        flags.append("--headless")

    if isinstance(scenario, Scenario):
        s = scenario.to_ron()
        print(s)
        subprocess.run([simulator, "scenario", "-", "-o", output] + flags, input=s, text=True)
    elif isinstance(scenario, str):
        subprocess.run([simulator, "scenario", scenario, "-o", output] + flags)
    else:
        print("Error: scenario must be a `Scenario` object or a string. Found: ", type(scenario))
        exit(1)


