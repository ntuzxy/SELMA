#include <OK_interface.h>
#include <pthread.h>
#include <unistd.h>
#include <cstring>

//#ifdef OK_CENTOS // for programmer/ide benefit
//#include "..\..\inc\OpalKelly\okFrontPanelDLL_Centos.h"
//#else
//#include "..\..\inc\OpalKelly\okFrontPanelDLL_Ubuntu.h"
//#endif
OkInterface* OkInterface::instance = NULL;
pthread_mutex_t OkInterface::ok_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t OkInterface::create_mutex = PTHREAD_MUTEX_INITIALIZER;

OkInterface* OkInterface::Instance() {
	if (!instance) {
		pthread_mutex_lock(&create_mutex);
		if (!instance) {
			instance = new OkInterface();
		}
		pthread_mutex_unlock(&create_mutex);
	}

	return instance;
}

OkInterface::OkInterface()
{
	printf("Initializing Opal Kelly XEM6010 board\n");

	if (!okFrontPanelDLL_LoadLib("libokFrontPanel.so"))
	{
		printf("Error loading .so file\n");
	}
	else
	{
		printf(".so file loaded!\n");
	}

	printf("opening device object\n");
	dev = new okCFrontPanel();

	printf("checking for physical device \n");
	while (dev->OpenBySerial("") != 0)
	{ 
		printf("Waiting for Opal Kelly connection\n");
	}

	if (dev->LoadDefaultPLLConfiguration() != 0)
	{
		printf("Error loading the PLL configuration\n");
	}

	//num_bytes_read = 0;
	//last_byte_location = 0;
	printf("Opal Kelly USB interface initialized\n");
}

okCFrontPanel::ErrorCode OkInterface::ConfigureFPGA(
		const std::string& bitfilePath) {
	if (access(bitfilePath.c_str(), F_OK) == -1) {
		printf("Error: The specified bitfile does not exist.\n %s\n",
				bitfilePath.c_str());
		printf(
				"Try running the following commands to download the latest bitfiles:\n");
		printf(
				"wget -O bitfiles.zip https://www.dropbox.com/sh/lkabdc33offdhzu/AAC7yIql-jdjyHTcQtH5-XUDa?dl=1\n");
		printf("unzip -o bitfiles.zip -d atis_bitfiles\n");
		printf("rm bitfiles.zip\n");
		return okCFrontPanel::ErrorCode::FileError;
	}

	return dev->ConfigureFPGA(bitfilePath.c_str());
}
void OkInterface::UpdateWireIns() {
	dev->UpdateWireIns();
}
okCFrontPanel::ErrorCode OkInterface::GetWireInValue(int epAddr, UINT32 *val) {
	return dev->GetWireInValue(epAddr, val);
}

okCFrontPanel::ErrorCode OkInterface::SetWireInValue(int ep, UINT32 val, UINT32 mask) {
	return dev->SetWireInValue(ep, val, mask);
}

void OkInterface::UpdateWireOuts() {
	dev->UpdateWireOuts();
}

unsigned long OkInterface::GetWireOutValue(int epAddr) {
	dev->GetWireOutValue(epAddr);
}

okCFrontPanel::ErrorCode OkInterface::ActivateTriggerIn(int epAddr, int bit) {
	return dev->ActivateTriggerIn(epAddr, bit);
}

void OkInterface::UpdateTriggerOuts() {
	dev->UpdateTriggerOuts();
}

bool OkInterface::IsTriggered(int epAddr, UINT32 mask){
	return dev->IsTriggered(epAddr, mask);
}

long OkInterface::WriteToPipeIn(int epAddr, long length, unsigned char *data) {
	dev->WriteToPipeIn(epAddr, length, data);
}

long OkInterface::ReadFromPipeOut(int epAddr, long length, unsigned char *data) {
	dev->ReadFromPipeOut(epAddr, length, data);
}

long OkInterface::WriteToBlockPipeIn(int epAddr, int blockSize, long length, unsigned char *data) {
	dev->WriteToBlockPipeIn(epAddr, blockSize, length, data);
}

long OkInterface::ReadFromBlockPipeOut(int epAddr, int blockSize, long length, unsigned char *data) {
	dev->ReadFromBlockPipeOut(epAddr, blockSize, length, data);
}
