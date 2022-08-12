#ifndef OK_INTERFACE_H
#define OK_INTERFACE_H
#include <okFrontPanelDLL.h>

//#ifdef OK_CENTOS // for programmer/ide benefit
//#include "OpalKelly\okFrontPanelDLL_Centos.h"
//#else
//#include "OpalKelly\okFrontPanelDLL_Ubuntu.h"
//#endif
// Singleton Wrapper around okCFrontPanel to provide Thread-safe access
// Specifically for the NP project as having this singleton will likely prevent
// us from using two or more ATIS at the same time (for stereo vision or whatever other reason)
// Also, singletons are anti-patterns but we use this as the quick and easy solution for the NP project.
// -Cedric
class OkInterface {
public:
	static pthread_mutex_t ok_mutex; // Used by any functions which want to communicate over the Opal Kelly USB
	static OkInterface* Instance();
	okCFrontPanel::ErrorCode ConfigureFPGA(const std::string& bitfilePath);
	void UpdateWireIns();
	okCFrontPanel::ErrorCode GetWireInValue(int epAddr, UINT32 *val);
	okCFrontPanel::ErrorCode SetWireInValue(int ep, UINT32 val, UINT32 mask = 0xffffffff);
	void UpdateWireOuts();
	unsigned long GetWireOutValue(int epAddr);
	okCFrontPanel::ErrorCode ActivateTriggerIn(int epAddr, int bit);
	void UpdateTriggerOuts();
	bool IsTriggered(int epAddr, UINT32 mask);
	long WriteToPipeIn(int epAddr, long length, unsigned char *data);
	long ReadFromPipeOut(int epAddr, long length, unsigned char *data);
	long WriteToBlockPipeIn(int epAddr, int blockSize, long length, unsigned char *data);
	long ReadFromBlockPipeOut(int epAddr, int blockSize, long length, unsigned char *data);
private:
	OkInterface(); // Prevent instances from being created except by itself
	OkInterface(OkInterface const&){}; // Copy constructor is private
	OkInterface& operator=(OkInterface const&){}; // prevent assignment
	static pthread_mutex_t create_mutex;
	static OkInterface* instance;
	okCFrontPanel *dev;		//UI Panel
};

#endif
