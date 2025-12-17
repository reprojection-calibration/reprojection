from setuptools import setup, find_packages
from setuptools.command.build_py import build_py as _build_py
import os
import subprocess


# TODO(Jack): Use setuptools-protobuf package

class BuildProto(_build_py):
    def run(self):
        # TODO(Jack): Is this a good way to get the file/paths?
        setup_path = os.path.dirname(__file__)
        proto_dir = os.path.join(setup_path, '../proto')
        if os.path.exists(proto_dir):
            for proto_file in os.listdir(proto_dir):
                if proto_file.endswith('.proto'):
                    proto_path = os.path.join(proto_dir, proto_file)
                    output_path = setup_path + '/database/'
                    print(proto_path)
                    subprocess.run(['protoc', '--python_out=', setup_path, '/database', proto_path])

        super().run()


setup(
    name="python_tooling",
    version="0.0.0",
    author="Jack Borer",
    author_email="reprojection-calibration@gmail.com",
    cmdclass={  # Hook to compile .proto files during the
        'build_py': BuildProto,
    },
    description=("A package for handling and visualization of calibration data."),
    license="GNU AGPL",
    url="https://github.com/reprojection-calibration/reprojection",
    packages=['database', 'tests'],
)
