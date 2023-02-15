#!/usr/bin/env python3
"""
Setup script for Edge Speech Intelligence System

Author: zwanderer
License: MIT
"""

from setuptools import setup, find_packages
import os

# Read README file
def read_readme():
    with open("README.md", "r", encoding="utf-8") as fh:
        return fh.read()

# Read requirements
def read_requirements():
    with open("requirements.txt", "r", encoding="utf-8") as fh:
        return [line.strip() for line in fh if line.strip() and not line.startswith("#")]

setup(
    name="edge-speech-to-intel",
    version="1.0.0",
    author="zwanderer",
    description="Passive embedded speech monitoring system for 24/7 deployment with RKNPU acceleration",
    long_description=read_readme(),
    long_description_content_type="text/markdown",
    url="https://github.com/zwanderer/edge-speech-to-intel",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: System Administrators",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Multimedia :: Sound/Audio :: Capture/Recording",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: System :: Hardware :: Embedded",
        "Topic :: Home Automation",
        "Topic :: Security",
    ],
    python_requires=">=3.8",
    install_requires=read_requirements(),
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "pytest-asyncio>=0.21.0",
            "black>=23.0.0",
            "flake8>=5.0.0",
            "mypy>=1.0.0",
        ],
        "monitoring": [
            "grafana-api>=1.0.3",
            "influxdb-client>=1.36.0",
            "prometheus-client>=0.16.0",
        ],
        "rknpu": [
            # RKNPU-specific packages when available
            # "rockchip-rknpu-toolkit>=1.0.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "edge-speech-intel=src.speech_processor:main",
        ],
    },
    keywords="edge computing, speech intelligence, VAD, RKNPU, ambient intelligence, passive monitoring",
    project_urls={
        "Bug Reports": "https://github.com/zwanderer/edge-speech-to-intel/issues",
        "Source": "https://github.com/zwanderer/edge-speech-to-intel",
        "Documentation": "https://github.com/zwanderer/edge-speech-to-intel/blob/main/README.md",
    },
    data_files=[
        ('share/edge-speech-intel/configs', ['docs/deployment_guide.md']),
        ('share/edge-speech-intel/esp32', ['esp32/README_PASSIVE_SENSORS.md']),
    ],
)