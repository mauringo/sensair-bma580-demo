#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
library_dir=${BMA580_LIBRARY_DIR:-.pio/libdeps/esp32-c5-sensairshuttle/BMA580}
if [ ! -f "$library_dir/src/BMA580.h" ]; then
    echo "BMA580 dependency missing. Run: pio run -e esp32-c5-sensairshuttle" >&2
    exit 1
fi
build_dir=$(mktemp -d /tmp/bma580-tests.XXXXXX)
trap 'rm -rf "$build_dir"' EXIT
for source in "$library_dir"/src/bosch/*.c; do
    cc -std=c99 -I "$library_dir/src/bosch" -c "$source" -o "$build_dir/$(basename "$source").o"
done
c++ -std=c++11 -Wall -Wextra -Werror -I include -I test/bma580/stubs -I "$library_dir/src" \
    test/bma580/test_driver.cpp "$library_dir/src/BMA580.cpp" "$build_dir/"*.o -lm -o "$build_dir/test"
"$build_dir/test"
