#!/bin/bash
#
# Requires uncrustify which, in turn, is built with cmake. See the following links:
#   https://github.com/uncrustify/uncrustify
#   https://cmake.org/install/
#

SCRIPT_PATH="${BASH_SOURCE[0]}";
if [ -h "${SCRIPT_PATH}" ]; then
  while([ -h "${SCRIPT_PATH}" ]) do SCRIPT_PATH=`readlink "${SCRIPT_PATH}"`; done
fi
pushd . > /dev/null
cd `dirname ${SCRIPT_PATH}` > /dev/null
SCRIPT_PATH=`pwd`;

cd $SCRIPT_PATH

for file in *.h *.cpp *.ino
do
  uncrustify --no-backup -c uncrustify.cfg $file
done

exit 0
