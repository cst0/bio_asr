#!/usr/bin/env python3

import rospkg
import os
import sys
import subprocess


def append_import_path(path):
    venv_packages = os.path.join(
        path,
        "venv/lib/python"
        + str(sys.version_info.major)
        + "."
        + str(sys.version_info.minor),
        "site-packages",
    )

    sys.path.append(venv_packages)


def attempt_imports(path):
    unresolved_dep = False
    for requirement in open(os.path.join(path, "requirements.txt"), "r").readlines():
        requirement = requirement.strip("\n\r ")
        if len(requirement) == 0:
            continue
        print("checking for " + requirement + "...")
        try:
            __import__(requirement)
        except:
            unresolved_dep = True
    return unresolved_dep


def attempt_dependency_resolution(path):
    for requirement in open(os.path.join(path, "requirements.txt"), "r").readlines():
        requirement = requirement.strip("\n\r ")
        if len(requirement) == 0:
            continue
        print("Resolving requirement for " + requirement + "...")
        cmd = os.path.join(path, "venv/bin/pip") + " install " + requirement
        subprocess.run(cmd, shell=True)


def main():
    rospack = rospkg.RosPack()
    rospack.list()
    path = rospack.get_path("bio_asr")
    os.chdir(path)

    if not os.path.exists(os.path.join(path, "venv")):
        print(
            "There's no virtual environment in your bio_asr package: we'll make one now."
        )
        subprocess.run("python3 -m venv venv", shell=True)

    append_import_path(path)

    unresolved_dep = attempt_imports(path)
    if unresolved_dep:
        print("unresolved dependencies")
        attempt_dependency_resolution(path)
        unresolved_dep = attempt_imports(path)
        if unresolved_dep:
            print("Tried resolving but it still didn't work...")
            sys.exit(1)
    print("Looks like everything is here!")


if __name__ == "__main__":
    main()
