# Modified cfclient files

These are GPL-licensed modifications of Bitcraze's cfclient.
They add incremental thrust: RT = thrust up, LT = thrust down,
Y = snap to hover, B = kill.

## Install
1. Copy `inputreaderinterface.py` to
   `.venv\Lib\site-packages\cfclient\utils\input\`
2. Rename `inputreaders__init__.py` to `__init__.py` and copy it to
   `.venv\Lib\site-packages\cfclient\utils\input\inputreaders\`
3. Copy `xbox_incremental.json` to
   `C:\Users\<you>\AppData\Local\Bitcraze\cfclient\input\`
4. Restart cfclient, then select Input device > xbox_incremental_controller

Hover value in IncrementalThrust is a placeholder (0.60). Measure it.