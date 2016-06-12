#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This is a setup.py script generated by py2applet

Usage:
    python setup.py py2app
"""

from setuptools import setup

APP = ['bed-leveling.py']
APP_NAME = "Bed Leveling"
DATA_FILES = []
OPTIONS = {
    'argv_emulation': True,
    'iconfile': 'icon.icns',
    'plist': {
        'CFBundleName': APP_NAME,
        'CFBundleDisplayName': APP_NAME,
        'CFBundleGetInfoString': "Marlin Firmware Mesh Bed Leveling Utility",
        'CFBundleIdentifier': "de.xythobuz.bed-leveling",
        'CFBundleVersion': "0.1.0",
        'CFBundleShortVersionString': "0.1.0",
        'NSHumanReadableCopyright': u"Copyright © 2016, Thomas Buck, All Rights Reserved"
    }
}

setup(
    app=APP,
    data_files=DATA_FILES,
    options={'py2app': OPTIONS},
    setup_requires=['py2app'],
)