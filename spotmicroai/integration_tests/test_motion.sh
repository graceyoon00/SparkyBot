#!/bin/bash

cd ~/SparkyBot/spotmicroai
export PYTHONPATH=.

venv/bin/python3 integration_tests/test_motion/test_motion.py
