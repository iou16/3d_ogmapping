#ifndef SMMAP_H
#define SMMAP_H
#include <../grid/map.h>
#include <../grid/harray3d.h>
#include <../utils/point.h>
#define SIGHT_INC 1

namespace ThreeDOGMapping {

	struct PointAccumulator{
		typedef point<float> FloatPoint;

		PointAccumulator(): acc(0,0,0), n(0), visits(0){}
		PointAccumulator(int i): acc(0,0,0), n(0), visits(0){assert(i==-1);}
		
		inline void update(bool value, const Point& p=Point(0,0,0));
		inline Point mean() const {return 1./n*Point(acc.x, acc.y, acc.z);}
		inline operator double() const { return visits?(double)n*SIGHT_INC/(double)visits:-1; }
		inline void add(const PointAccumulator& p) {acc=acc+p.acc; n+=p.n; visits+=p.visits; }
		static const PointAccumulator& Unknown();
		static PointAccumulator* unknown_ptr;
		FloatPoint acc;
		int n, visits;
		inline double entropy() const;

	};

	void PointAccumulator::update(bool value, const Point& p){
		if (value) {
			acc.x+= static_cast<float>(p.x);
			acc.y+= static_cast<float>(p.y);
			acc.z+= static_cast<float>(p.z);
			n++;
			visits+=SIGHT_INC;
		} else
			visits++;
	}

	double PointAccumulator::entropy() const {
		if(!visits)
			return -log(.5);
		if (n==visits || n==0)
			return 0;
		double x=(double)n*SIGHT_INC/(double)visits;
		return -( x*log(x) + (1-x)*log(1-x) );
	}

	typedef Map< PointAccumulator, HierarchicalArray3D<PointAccumulator> > ScanMatcherMap;

};

#endif
