from setuptools import setup, find_packages
import codecs
import urllib.request
import json


def parse_requirements(filename):
    with open(filename, "r") as f:
        return [line.strip() for line in f if line.strip() and not line.startswith("#")]


def get_latest_release_version():
    url = f"https://api.github.com/repos/PIDTuningIreland/pyPIDTune/releases/latest"
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    response = urllib.request.urlopen(req)
    if response.getcode() == 200:
        release_info = json.loads(response.read())
        return release_info["tag_name"]
    else:
        raise Exception("Cannot Find Version")


with codecs.open("README.md", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="pypidtune",
    version=get_latest_release_version(),
    license="MIT",
    author="PIDTuningIreland",
    author_email="pidtuningireland@gmail.com",
    description="PID tuner, logger, simulator and process emulator",
    long_description=long_description,
    long_description_content_type="text/markdown",
    packages=find_packages(),
    url="https://github.com/PIDTuningIreland/pypidtune",
    keywords="PID Tuner",
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Information Technology",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Operating System :: MacOS :: MacOS X",
        "Operating System :: Microsoft :: Windows",
    ],
    install_requires=parse_requirements("requirements.txt"),
)
