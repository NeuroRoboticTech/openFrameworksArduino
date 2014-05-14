-- A solution contains projects, and defines the available configurations
solution "openFrameworksArduino_x64"
	configurations { "Debug", "Release" }
	platforms {"x64"}

	project "openFrameworksArduino_x64"
		language "C++"
		kind     "SharedLib"
		files  { "../src/*.h",
				 "../src/*.cpp"}
		includedirs { "../../boost_1_54_0" }	  
		libdirs { "../../boost_1_54_0/lib_x64" }
		
		configuration { "Debug", "windows" }
			defines { "WIN32", "_DEBUG", "_WINDOWS", "_USRDLL", "_CRT_SECURE_NO_WARNINGS" }
			flags   { "Symbols", "SEH" }
			targetdir ("Debug")
			targetname ("openFrameworksArduinoD_x64")
	 
		configuration { "Release", "windows" }
			defines { "WIN32", "NDEBUG", "_WINDOWS", "_USRDLL" }
			flags   { "Optimize", "SEH" }
			targetdir ("Release")
			targetname ("openFrameworksArduino_x64")
			
	project "ArduinoTest_x64"
		language "C++"
		kind     "ConsoleApp"
		files  { "../examples/*.h",
				 "../examples/*.cpp"}
		includedirs { "../../boost_1_54_0",
					  "../src" }	  
		libdirs { "../../boost_1_54_0/lib_x64" }
		
		configuration { "Debug", "windows" }
			defines { "WIN32", "_DEBUG", "_WINDOWS", "_USRDLL", "_CRT_SECURE_NO_WARNINGS" }
			flags   { "Symbols", "SEH" }
			targetdir ("Debug")
			targetname ("ArduinoTestD_x64")
			links { "openFrameworksArduino_x64" }
	 
		configuration { "Release", "windows" }
			defines { "WIN32", "NDEBUG", "_WINDOWS", "_USRDLL" }
			flags   { "Optimize", "SEH" }
			targetdir ("Release")
			targetname ("ArduinoTest_x64")
			links { "openFrameworksArduino_x64" }
