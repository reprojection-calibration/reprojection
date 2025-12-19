from setuptools import setup

setup(
    name="python_tooling",
    version="0.0.0",
    author="Jack Borer",
    author_email="reprojection-calibration@gmail.com",
    description=(
        "A package for handling and visualization of calibration data."),
    license="GNU AGPL",
    url="https://github.com/reprojection-calibration/reprojection",
    packages=['database', 'generated', 'tests'],
)
