-- A solution contains projects, and defines the available configurations
solution "openFrameworksArduino"
	configurations { "Debug", "Release" }

	project "openFrameworksArduino"
		language "C++"
		kind     "SharedLib"
		files  { "../src/*.h",
		         "../src/*.cpp"}
		buildoptions { "-std=c++11" }
		links { "dl" }
		
		configuration { "Debug", "linux" }
			defines { "_DEBUG" }
			flags   { "Symbols", "SEH" }
			targetdir ("Debug")
			targetname ("openFrameworksArduinoD")
	 
		configuration { "Release", "linux" }
			defines { "NDEBUG" }
			flags   { "Optimize", "SEH" }
			targetdir ("Release")
			targetname ("openFrameworksArduino")
			
	project "ArduinoTest"
		language "C++"
		kind     "ConsoleApp"
		files  { "../examples/*.h",
				 "../examples/*.cpp"}
		includedirs { "../src" }	  
		
		configuration { "Debug", "linux" }
			defines { "_DEBUG" }
			flags   { "Symbols", "SEH" }
			targetdir ("Debug")
			targetname ("ArduinoTestD")
			links { "openFrameworksArduino", "boost_thread", "boost_system" }
	 
		configuration { "Release", "linux" }
			defines { "NDEBUG" }
			flags   { "Optimize", "SEH" }
			targetdir ("Release")
			targetname ("ArduinoTest")
			links { "openFrameworksArduino", "boost_thread", "boost_system" }
