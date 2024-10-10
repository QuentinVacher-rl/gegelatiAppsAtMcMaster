#define _USE_MATH_DEFINES // To get M_PI
#include <math.h>

#include "mujocoWrapper.h"


std::vector<std::reference_wrapper<const Data::DataHandler>> MujocoWrapper::getDataSources()
{
	auto result = std::vector<std::reference_wrapper<const Data::DataHandler>>();
	result.push_back(this->currentState);
	return result;
}

