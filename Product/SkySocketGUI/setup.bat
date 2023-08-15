@ECHO OFF
move /y SkySocketFiles %UserProfile%\Desktop\SkySocketFiles
icacls %UserProfile%\Desktop\SkySocketFiles /grant Everyone:F
curl -o SkySocketPython.zip -url https://www.python.org/ftp/python/3.11.4/python-3.11.4-embed-amd64.zip
mkdir SkySocketPython
tar -xf SkySocketPython.zip -C SkySocketPython
move /y res\installer\python311._pth SkySocketPython\
curl -o get-pip.py -url https://bootstrap.pypa.io/get-pip.py 
del SkySocketPython.zip
SkySocketPython\python.exe get-pip.py --no-warn-script-location
del get-pip.py
SkySocketPython\python.exe -m pip install -r res\installer\requirements.txt --no-warn-script-location
copy /y res\skysockettheme\skysockettheme-0.png SkySocketPython\Lib\site-packages\kivy\data\images\defaulttheme-0.png
copy /y res\skysockettheme\skysockettheme.atlas SkySocketPython\Lib\site-packages\kivy\data\images\defaulttheme.atlas
SkySocketPython\python.exe -m compileall .