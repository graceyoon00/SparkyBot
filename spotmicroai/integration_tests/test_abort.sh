#!/bin/bash

cd ~/SparkyBot/spotmicroai
export PYTHONPATH=.

venv/bin/python3 integration_tests/test_abort/test_abort.py

