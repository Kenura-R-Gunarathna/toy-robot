# Usage: source scripts/env.sh   (bash/zsh)
_root="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
export IDF_PATH="$_root/libs/espressif/esp-idf"
export IDF_CCACHE_ENABLE=1
. "$IDF_PATH/export.sh"
unset _root
