@echo off

set SBGC_RUN_CMD=java -Dsimplebgc_gui.SimpleBGC_GUIView.Logger.level=0  -Dlog4j.configuration=log4j.properties -Dgnu.io.rxtx.NoVersionOutput=true -Djava.library.path=./lib -classpath ./lib/;SimpleBGC_GUI.jar  simplebgc_gui.SimpleBGC_GUICmdApp

%SBGC_RUN_CMD%  %*


