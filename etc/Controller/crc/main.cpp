#include "includes/crc.hpp"

int main(int argc, char *argv[]){
	CentralResiliencyCoordinator::CentralRC CRC;
	
	gettimeofday(&CRC.launch_time_, NULL);
	while(CRC.is_node_running_){
		CRC.communicate();
	}

	return 0;
}
