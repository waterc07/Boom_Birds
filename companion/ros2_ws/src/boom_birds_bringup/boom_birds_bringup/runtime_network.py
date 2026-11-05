"""Temporary DDS socket limits for the owned sensor runtime."""
import os
import subprocess
import sys


class DdsSocketBuffers:
    def __init__(self, read=None, write=None):
        self.read = read or self.readValue
        self.write = write or self.writeValue
        self.original = {}
        self.minimum = 4 * 1024 * 1024

    @staticmethod
    def readValue(name):
        return int(subprocess.check_output(["sysctl", "-n", name], text=True).strip())

    @staticmethod
    def writeValue(name, value):
        prefix = [] if os.geteuid() == 0 else ["sudo", "-n"]
        command = ["sysctl", "-w", f"{name}={value}"]
        result = subprocess.run(prefix + command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode and prefix and "password is required" in result.stderr and sys.stdin.isatty():
            subprocess.run(["sudo"] + command, check=True)
        elif result.returncode:
            raise RuntimeError("DDS socket buffer: " + result.stderr.strip())

    def prepare(self):
        for name in ("net.core.rmem_max", "net.core.wmem_max"):
            previous = self.read(name)
            if previous < self.minimum:
                self.original[name] = previous
                self.write(name, self.minimum)

    def restore(self):
        for name, previous in self.original.items():
            if self.read(name) == self.minimum:
                self.write(name, previous)
