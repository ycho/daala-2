#!/bin/bash -e
# continuous integration test script
# run this from the top-level source directory

make -C unix check
make -C unix clean
make -C tools/unix
make -C tools/unix clean
DAALA_ROOT=. tools/submit_awcy.py -master -branch master
