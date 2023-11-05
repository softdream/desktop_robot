#include "localize.h"


int main()
{
	std::cout<<"------------------------ LOCALIZATION TEST -------------------------"<<std::endl;

	chassis::Localize<float> localize_instance;
	localize_instance.run();

	return 0;
}
