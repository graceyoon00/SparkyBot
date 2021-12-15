#!/bin/bash

cd ~/SparkyBot/spotmicroai || exit
export PYTHONPATH=.

venv/bin/python3 ~/SparkyBot/spotmicroai/calibration/calibration/calibration.py
