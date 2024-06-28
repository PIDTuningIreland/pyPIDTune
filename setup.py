from setuptools import setup, find_packages
import codecs


def parse_requirements(filename):
    with open(filename, "r") as f:
        return [line.strip() for line in f if line.strip() and not line.startswith("#")]


with codecs.open("README.md", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="pypidtune",
    version="0.0.01",
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
