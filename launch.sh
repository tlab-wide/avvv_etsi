#!/usr/bin/env sh

SCRIPT_DIR=$(dirname "$BASH_SOURCE")
SCRIPT_PATH=$(readlink -f $SCRIPT_DIR)
export AVVV_ETSI_HOME="$SCRIPT_PATH"/

source "$AVVV_ETSI_HOME/lib/avvv_etsi_venv/bin/activate"
source "$AVVV_ETSI_HOME/visualiser/install/setup.bash"

"$AVVV_ETSI_HOME"ui/visually_launcher/build/visually_launcher

deactivate