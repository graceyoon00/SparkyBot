#!/bin/bash

cd ~/SparkyBot/spotmicroai
export PYTHONPATH=.

venv/bin/python3 ~/SparkyBot/spotmicroai/integration_tests/test_remote/test_remote.py

