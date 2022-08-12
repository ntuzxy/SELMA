#/bin/sh

#for legacy support of display functions for tracking and orientation estimation. OpenCV must be installed. Soon to be deprecated
useOpenCV=false

#to use display functions for APS and TD events. Setting to false will allow compilation withough SDL being installed. Useful when visual display is not required
useSDL=true

#include DAVIS interfaces? Setting to false will allow compilation without libcaer installed, but then DAVIS cannot be used
useDAVIS=false

#include ATIS. Setting to false will allow compilation without installation of Opal Kelly drivers. Useful if you have no intention of using ATIS
useATIS=true

#do we want to enable debugging???
useDebug=true

#include TN Runtime. Setting to true will require the presence of tn-runtime libraries.  
useTN=false

#include SELMA_Classifier. Setting to false will allow compilation without SELMA
useSELMA=true

arch="$(dpkg --print-architecture)" #check for ARM architecture


if [ "$useATIS" = true ] ; then
	atisFLAG="-D _useATIS"
else
	atisFLAG=""
fi

if [ "$useDAVIS" = true ] ; then
	davisFLAG="-D _useDAVIS -I/usr/include/libcaer/"
	davisLINK="-lcaer"
else
	davisFLAG=""
	davisLINK=""
fi

if [ "$useSDL" = true ] ; then
	sdlFLAG="-D _useSDL"
else
	sdlFLAG=""
fi

# check which OS we're using and copy the corresponding frontpanel header accordingly
OS=$(lsb_release -si)
if [[ $OS == *"CentOS"* ]]; then
	echo "CentOS detected. The Opal Kelly header file is for CentOS 6.5"	
	cp inc/OpalKelly/okFrontPanelDLL_Centos.h inc/okFrontPanelDLL.h
	g_plus_plus="g++ -march=native -mtune=native"
else
	if [[ $OS == *"Ubuntu"* ]]; then
		echo "Ubuntu detected. The Opal Kelly header file is for Ubuntu 12.04"
		cp inc/OpalKelly/okFrontPanelDLL_Ubuntu.h inc/okFrontPanelDLL.h

		OSversion=$(lsb_release -rs)
		if [[ $OSversion == *"16.04"* ]]; then
			echo "Ubuntu 16.04 detected. g++ version 4.9 must be used. Please install if not there"
		 	#g_plus_plus="g++-4.9"
		 	g_plus_plus="g++ -march=native -mtune=native"
		else
			g_plus_plus="g++ -march=native -mtune=native"
			#g_plus_plus="g++"
		fi		

	else
		echo "ERROR: OS not detected!!!"	
		exit 1
	fi
fi

if [ "$useDebug" = true ] ; then
	debugLINK="-g"
	g_plus_plus=$g_plus_plus" -ggdb "
else
	debugLINK=""
fi

if [[ "${arch}" =~ 'arm' ]]; then
	g_plus_plus=$g_plus_plus" -mfloat-abi=hard -mfpu=neon -ftree-vectorize "
fi

if [ "$useATIS" = true ]; then
	if [ ! -d atis_bitfiles ]; then
		echo "FPGA bitfiles not detected. Will proceed to download the latest version"
		wget -O bitfiles.zip https://www.dropbox.com/sh/lkabdc33offdhzu/AAC7yIql-jdjyHTcQtH5-XUDa?dl=1
		unzip -o bitfiles.zip -d atis_bitfiles
		rm bitfiles.zip
	fi
fi

# check that the required directories exist
if [ ! -d executable ]; then
	echo "Making executable directory"	
	mkdir executable
fi	
if [ ! -d outputs ]; then
	echo "Making output directory"	
	mkdir outputs
fi

#TAKE CARE OF INCLUDING OpenCV. THIS IS OPTIONAL AND CONTROLLED BY THE "useOpenCV" FLAG
if [ "$useOpenCV" = true ] ; then
	echo "OpenCV flag set to TRUE, will attempt to compile with OPENCV"
	# look for common opencv directories... probably a better way to do this
	if [ -d "/usr/local/include/opencv2" ]; then
		opencv_path="/usr/local/include/opencv2"
	else
		if [ -d "/usr/include/opencv2" ]; then
			opencv_path="/usr/include/opencv2"
		else
			echo "opencv directory not found"	
			exit 1
		fi
	fi
	echo "Using opencv directory: $opencv_path"
	cvFLAG="-D _useOPENCV -I$opencv_path -I$opencv_path/highgui"
	cvLINK="`pkg-config --cflags opencv`  `pkg-config --libs opencv`"
else
	cvFLAG=""
	cvLINK=""
 	echo "OpenCV flag set to FALSE, functions relying on OPENCV will be omitted"
fi


#TAKE CARE OF INCLUDING SDL FOR DISPLAY. THIS IS OPTIONAL AND CONTROLLED BY THE "useSDL" FLAG
if [ "$useSDL" = true ] ; then
	echo "SDL flag set to TRUE, will attempt to compile with SDL"
	export LD_LIBRARY_PATH="/usr/local/lib"
	sdlFLAG="-w -lSDL2 -D _useSDL `pkg-config --cflags sdl2`"
	sdlLINK="-w -lSDL2"
else
	echo "SDL flag set to FALSE, basic display functions will not be available"
	sdlFLAG=""
	sdlLINK=""
fi


set -e #set the script to stop on error
echo "Compiling FILE interface"
$g_plus_plus -c -o outputs/FILE_interface.o src/event_sources/FILE_interface.cpp -I./inc/ -std=c++0x -O3

if [ "$useATIS" = true ] ; then
	echo "Compiling ATIS interface"
	$g_plus_plus -c -o outputs/OK_interface.o src/event_sources/OK_interface.cpp -I./inc/ -std=c++0x
	$g_plus_plus -c -o outputs/ATIS_interface.o src/event_sources/ATIS_interface.cpp -I./inc/ -std=c++0x
	atisOutput="outputs/OK_interface.o outputs/ATIS_interface.o /usr/local/lib/libokFrontPanel.so"
else
	echo "ATIS interface disabled, skipping Opal Kelly and ATIS code"
	atisOutput=""
fi

if [ "$useDAVIS" = true ] ; then
	echo "Compiling DAVIS240c interface"
	$g_plus_plus -c -o outputs/DAVIS_interface.o src/event_sources/DAVIS_interface.cpp -I./inc/ $davisFLAG -std=c++0x -O3
	davisOutput="outputs/DAVIS_interface.o"
else
	davisOutput=""
fi

tnFLAG=""
if [ "$useTN" = true ] ; then

	if [[ "${arch}" =~ 'arm' ]]; then
		echo "ARM architecture"
        tnFLAG="-pthread -L/usr/local/lib/tn-runtime/ -ltnmetal -ltnbaremetal"
        tnRuntimeDir="/usr/local/include"
        tnLINK="-L/usr/local/lib/tn-runtime/ -ltnmetal -ltnbaremetal"
    else
    	echo "non-ARM architecture"
	    tnRuntimeDir="inc"
	    tnLINK=""
	fi

	tnFLAG=$tnFLAG" -D _useTN -I"$tnRuntimeDir"/tn-runtime/ -I"$tnRuntimeDir"/tn-runtime/tn-tools/"
else
	tnFLAG=""
	tnLINK=""
fi

echo "Compiling Protobuf Files"
protoc -I=src/protobuf/ --cpp_out=src/protobuf/ src/protobuf/events.proto
protoc -I=src/protobuf/ --cpp_out=src/protobuf/ src/protobuf/tn_message.proto


if [ "$useSELMA" = true ]; then
	echo "Compiling SELMA Classifier"
	selmaFLAG="-D _useSELMA";
	$g_plus_plus -c $debugLINK $sdlFLAG -o outputs/SELMA_classifier.o src/SELMA_classifier.cpp -I./inc/ $selmaFLAG $atisFLAG -lboost_system -lprotobuf -std=c++0x -O3
	# # selmaOutput="outputs/SELMA_classifier.o"
	$g_plus_plus -c $debugLINK $sdlFLAG -o outputs/SELMA_IF.o src/SELMA_IF.cpp -I./inc/ $selmaFLAG $atisFLAG -std=c++0x -O3
	# selmaOutput="outputs/SELMA_IF.o"

	
	selmaOutput="outputs/SELMA_classifier.o outputs/SELMA_IF.o"
else
	selmaFLAG=""
	selmaOutput=""	
fi

echo "Compiling file writing functions"
$g_plus_plus -c -o outputs/file_writing.o src/file_writing.cpp -I./inc/ -std=c++0x -O3

echo "Compiling AER display functions"
$g_plus_plus -c $sdlFLAG -o outputs/AER_display.o src/AER_display.cpp $cvFLAG -I./inc/ $davisFLAG -std=c++0x -O3

echo "Compiling Filtering functions 1"
$g_plus_plus -c $sdlFLAG -o outputs/events.pb.o src/protobuf/events.pb.cc -I./inc/ -I/usr/local/include -std=c++0x -O3 -lprotobuf
$g_plus_plus -c $sdlFLAG -o outputs/tn_message.pb.o src/protobuf/tn_message.pb.cc -I/usr/local/include -I./inc/ -std=c++0x -O3 -lprotobuf
$g_plus_plus -c $sdlFLAG -o outputs/evt_tcp_publisher.o src/evt_tcp_publisher.cpp -I/usr/local/include -I./inc/  $davisFLAG $tnFLAG $atisFLAG -std=c++0x -O3 -lboost_system -lprotobuf

$g_plus_plus -c $sdlFLAG -o outputs/TN_classifier.o src/TN_classifier.cpp -I./inc/ $tnFLAG $atisFLAG -std=c++0x -Ofast -lboost_system -lprotobuf -lpthread
tnOutput="outputs/TN_classifier.o"  

echo "Compiling Filtering functions 2"
$g_plus_plus -c $sdlFLAG -o outputs/filters.o src/filters.cpp -I./inc/  $davisFLAG $tnFLAG $atisFLAG -std=c++0x -O3 -lboost_system

echo "Compiling Tracking functions"
$g_plus_plus -c $sdlFLAG -o outputs/tracking.o src/tracking.cpp $cvFLAG -I./inc/  $davisFLAG -std=c++0x -O3

echo "Compiling Thread Launcher interface"
$g_plus_plus -c $sdlFLAG $atisFLAG -o outputs/thread_launcher.o src/thread_launcher.cpp -I./inc/ $davisFLAG -std=c++0x -O3

echo "Compiling Main function"
$g_plus_plus -c $sdlFLAG -o outputs/epf.o src/event_processing_framework.cpp $davisFLAG $tnFLAG $selmaFLAG -I/usr/local/include -I./inc/ -std=c++0x -Os

echo "Linking"
$g_plus_plus $debugLINK -pthread -o executable/epf outputs/epf.o outputs/file_writing.o outputs/AER_display.o outputs/tracking.o $atisOutput outputs/events.pb.o outputs/tn_message.pb.o outputs/evt_tcp_publisher.o $tnOutput $selmaOutput outputs/filters.o outputs/FILE_interface.o $davisOutput outputs/thread_launcher.o -std=c++0x -ldl $sdlLINK $davisLINK $cvLINK $tnLINK -L/usr/local/lib -lboost_system -lprotobuf

#outputs/Bias_interface.o
#outputs/source_interface.o
#to debug you would run:
#sudo gdb --args ./epf -davis240C -D_TD
#then type run

echo "Done"
