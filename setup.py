from setuptools import setup, find_packages

setup(
    name = "uArmForPython",
    version = "0.1",
    packages = find_packages(),
    install_requires = ['pyserial>=2.4'],
    author = "laboratorio",
    author_email = "yichen@evol.net",
    description = "Python bindings for the uArm Firmata protocol",
    # license = "GPL v3",
    # keywords = "arduino firmata microcontroller ubicomp",
    url = "https://github.com/yichenpan/uArmForPython"
    # long_description="The aim of this project is to allow developers to communicate with Arduino microcontrollers using Python.",
)