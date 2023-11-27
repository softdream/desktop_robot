#include "data_type.h"

#include "data_transport.h"

#include <unistd.h>

int main()
{
	transport::Receiver udp_srv( LocalizationProcessRecverPort );

	int cnt = 0;
	while ( 1 ) {
		sensor::MessageType msg = sensor::IsKeyPose;
                auto ret = udp_srv.send( msg, "127.0.0.1", BehaviorPlannerProcessRecverPort );
		std::cout<<"send  : "<<ret<<std::endl;

		usleep( 500000 );
	
	
		cnt ++;

		if ( cnt == 20 ) { 
			sensor::MessageType msg = sensor::ArriveGoalPose;
			auto ret = udp_srv.send( msg, "127.0.0.1", BehaviorPlannerProcessRecverPort );
                	std::cout<<"send  : "<<ret<<std::endl;
			
		}

		if ( cnt == 40 ) {
			sensor::MessageType msg = sensor::ArriveGoalYaw;
                        auto ret = udp_srv.send( msg, "127.0.0.1", BehaviorPlannerProcessRecverPort );
                        std::cout<<"send  : "<<ret<<std::endl;
		
			break;
		}
	}

	return 0;
}
