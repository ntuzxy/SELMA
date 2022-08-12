This folder contains bitfiles commonly used with ATIS. The file name convention is: 
ATIS VERSION _ FPGA VERSION _ ADDITIONAL MODULES

If you see a bitfile with ADDITIONAL MODULES you would like to use, but it is not available for your version of ATIS and/or FPGA, then let me (Garrick) know and I'll generate it for you. 
garrickorchard@gmail.com



For use with Windows: 

Rename the bitfile you'd like to use to "ramtest.bit" and copy it to the same directory as your ATISviewer.exe executable.



For use with Linux there are two options: 

1: Rename the bitfile you'd like to use to "v6default.bit" and make sure it is in the "bitfiles" directory. The linux interface will then use this bitfile by default. If you are using ATIS version 4, you need to name the file "v4default.bin" and start atis with the switch -v4.

2: Manually specify the bitfile to use when starting ATIS using the optional command "-bitfile path_to_bitfile/bitfile_name". ie. 
$./atis -bitfile path_to_bitfile/bitfile_name
for v4 it is still necessary to use the -v4 switch so the software knows which ATIS version is being used.



Description of bitfiles is below:

v4:
Means ATIS version 4, which has external bias generator chips. It can be identified by the presence of two 8mmx8mm chips (DACs) on the back of the ATIS PCB.

v6:
Means ATIS version 6, which has internal bias generator. It can be identified by the lack of two 8mmx8mm chips (DACs) on the back of the ATIS PCB.

lx45:
Means the bitfile is for the Opal Kelly XEM6010lx45 board

lx150:
Means the bitfile is for the Opal Kelly XEM6010lx150 board







TrueNorth and Filter are now included by default in v6_lx150, but can be enabled/disabled in software.
TN:
Means the bitfile includes a TrueNorth interface for TD events. If you need the TrueNorth code to support this, please contact me.

Filter:
Means the bitfile includes an filter on the FPGA, which implement a nearest neighbout filter with 50ms threshold, and a refractory period filter with 1ms threshold. For details of the filter, see:
Czech, D.; and Orchard, G. "Evaluating Noise Filtering for Event-based Asynchronous Change Detection Image Sensors" 6th IEEE RAS & EMBS International Conference on Biomedical Robotics and Biomechatronics (BioRob), Singapore, June 2016 