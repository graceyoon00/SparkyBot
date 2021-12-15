#!/bin/bash

cd ~/SparkyBot/spotmicroai || exit

git reset --hard HEAD
git clean -df
git checkout master
git pull

find . -type f -iname "*.sh" -exec chmod +x {} \;

~/SparkyBot/spotmicroai/utilities/activate.sh
