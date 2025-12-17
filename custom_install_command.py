from setuptools import setup, find_packages
from setuptools.command.install import install
from grpc_tools import protoc

class CustomInstallCommand(install):
    def run(self):
        # Run protoc to generate Python files from proto files
        protoc.main((
            '',
            '-I=proto',  # Path to .proto files
            '--python_out=my_package',  # Output directory for Python files
            '--grpc_python_out=my_package',  # Output directory for gRPC files (if using gRPC)
            'proto/example.proto',  # Path to .proto file(s)
        ))
        install.run(self)

setup(
    name="my_project",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        'protobuf',
        'grpcio-tools',
    ],
    cmdclass={
        'install': CustomInstallCommand,
    },
)