#!/usr/bin/env python

from setuptools import setup

setup(
    name='UArmForPython',
    version='1.0.1',  # Use bumpversion!
    description="A Python library For UArm",
    author='Joey Song',
    author_email='astainsong@gmail.com',
    packages=['UArmForPython'],
    include_package_data=True,
    install_requires=['pyfirmata'],
    zip_safe=False,
    url='https://github.com/uArm-Developer/UArmForPython',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Other Environment',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.3',
        'Programming Language :: Python :: 3.4',
        'Topic :: Utilities',
        'Topic :: Home Automation',
    ],
)
