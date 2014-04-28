-- A solution contains projects, and defines the available configurations
solution "openFrameworksArduino"
	configurations { "Debug", "Release" }
	platforms {"x32"}

	project "openFrameworksArduino"
		language "C++"
		kind     "SharedLib"
		files  { "../src/*.h",
				 "../src/*.cpp"}
		includedirs { "../../boost_1_54_0" }	  
		libdirs { "../../boost_1_54_0/lib" }
		
		configuration { "Debug", "windows" }
			defines { "WIN32", "_DEBUG", "_WINDOWS", "_USRDLL", "_CRT_SECURE_NO_WARNINGS" }
			flags   { "Symbols", "SEH" }
			targetdir ("Debug")
			targetname ("openFrameworksArduinoD")
	 
		configuration { "Release", "windows" }
			defines { "WIN32", "NDEBUG", "_WINDOWS", "_USRDLL" }
			flags   { "Optimize", "SEH" }
			targetdir ("Release")
			targetname ("openFrameworksArduino")
			
	project "ArduinoTest"
		language "C++"
		kind     "ConsoleApp"
		files  { "../examples/*.h",
				 "../examples/*.cpp"}
		includedirs { "../../boost_1_54_0",
					  "../src" }	  
		libdirs { "../../boost_1_54_0/lib" }
		
		configuration { "Debug", "windows" }
			defines { "WIN32", "_DEBUG", "_WINDOWS", "_USRDLL", "_CRT_SECURE_NO_WARNINGS" }
			flags   { "Symbols", "SEH" }
			targetdir ("Debug")
			targetname ("ArduinoTestD")
			links { "openFrameworksArduino" }
	 
		configuration { "Release", "windows" }
			defines { "WIN32", "NDEBUG", "_WINDOWS", "_USRDLL" }
			flags   { "Optimize", "SEH" }
			targetdir ("Release")
			targetname ("ArduinoTest")
			links { "openFrameworksArduino" }
