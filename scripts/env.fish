# Usage: source scripts/env.fish
set -l root (realpath (dirname (status filename))/..)
set -gx IDF_PATH $root/libs/espressif/esp-idf
set -gx IDF_CCACHE_ENABLE 1
source $IDF_PATH/export.fish
