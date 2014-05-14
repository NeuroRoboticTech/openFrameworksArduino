echo "Generating openFrameworksArduino Solutions"

..\..\..\AnimatLabPublicSource\bin\premake4 --os=windows --file=openFrameworksArduino.lua vs2010
..\..\..\AnimatLabPublicSource\bin\premake4 --os=windows --file=openFrameworksArduino_x64.lua vs2010

@pause