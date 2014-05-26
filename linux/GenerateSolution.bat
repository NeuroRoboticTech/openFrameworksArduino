echo "Generating openFrameworksArduino Solutions"

..\..\..\AnimatLabPublicSource\bin\premake4 --os=linux --file=openFrameworksArduino.lua codeblocks
..\..\..\AnimatLabPublicSource\bin\premake4 --os=linux --file=openFrameworksArduino.lua gmake

@pause