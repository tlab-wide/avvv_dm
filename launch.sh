#!/usr/bin/env sh

SCRIPT_DIR=$(dirname "$BASH_SOURCE")
SCRIPT_PATH=$(readlink -f $SCRIPT_DIR)
export AVVV_DM_HOME="$SCRIPT_PATH"/

source "$AVVV_DM_HOME/lib/avvv_dm_venv/bin/activate"
source "$AVVV_DM_HOME/visualiser/install/setup.bash"

"$AVVV_DM_HOME"ui/visually_dm_launcher/build/visually_launcher

deactivate