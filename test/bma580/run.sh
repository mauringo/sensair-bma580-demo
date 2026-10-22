#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
build_dir=$(mktemp -d /tmp/bma580-tests.XXXXXX)
trap 'rm -rf "$build_dir"' EXIT
for source in lib/BMA580/src/bosch/*.c; do
    cc -std=c99 -I lib/BMA580/src/bosch -c "$source" -o "$build_dir/$(basename "$source").o"
done
c++ -std=c++11 -Wall -Wextra -Werror -I include -I test/bma580/stubs -I lib/BMA580/src \
    test/bma580/test_driver.cpp lib/BMA580/src/BMA580.cpp "$build_dir/"*.o -lm -o "$build_dir/test"
"$build_dir/test"
