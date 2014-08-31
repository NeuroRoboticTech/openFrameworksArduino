-- A solution contains projects, and defines the available configurations
solution "openFrameworksArduino"
	configurations { "Debug", "Release" }
	platforms {"x32"}

	project "openFrameworksArduino"
		language "C++"
		kind     "SharedLib"
		files  { "../src/*.h",
				 "../src/*.cpp"}
		includedirs { "$(BOOST_ROOT)" }	  
		libdirs { "$(BOOST_ROOT)/lib" }
		
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
		includedirs { "$(BOOST_ROOT)",
					  "../src",
					  "$(OSG_ROOT)/include" }	  
		libdirs { "$(BOOST_ROOT)/lib",
				  "$(OSG_ROOT)/lib" }
		
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
