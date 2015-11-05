#include "smmap.h"

namespace ThreeDOGMapping {

const PointAccumulator& PointAccumulator::Unknown(){
	if (! unknown_ptr)
		unknown_ptr=new PointAccumulator;
	return *unknown_ptr;
}

PointAccumulator* PointAccumulator::unknown_ptr=0;

};


