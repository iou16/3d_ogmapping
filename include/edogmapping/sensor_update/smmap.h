#ifndef SMMAP_H
#define SMMAP_H
#include <../map/map.h>
#include <../map/harray2d.h>
#include <../utils/point.h>
#define SIGHT_INC 1

namespace EDOGMapping {

	struct PointAccumulator{
		typedef point<float> FloatPoint;

		PointAccumulator(): acc(0,0), visits(0){}
		PointAccumulator(int i): acc(0,0), visits(0){assert(i==-1);}
		
		inline void update(double value, const Point& p=Point(0,0));
	    inline void gupdate(double value);
		inline Point pmean() const { return 1./visits*Point(acc.x, acc.y); }
		// inline operator double() const { return visits?diff/(double)visits:-1; }
        inline double mean() const { return visits?acc_diff/(double)visits:-1; }
        inline double cov() const { return visits?(acc_diff_sq/(double)visits)-(acc_diff/(double)visits):-1; }
        inline double gmean() const { return g_visits?g_acc_diff/(double)g_visits:-1; }
		inline void add(const PointAccumulator& p) {acc=acc+p.acc;/* n+=p.n;*/ visits+=p.visits; }
		static const PointAccumulator& Unknown();
		static PointAccumulator* unknown_ptr;
		FloatPoint acc;
        double acc_diff=0.0, acc_diff_sq=0.0, g_acc_diff=0.0;
        int visits=0,g_visits=0;
	};

	void PointAccumulator::update(double value, const Point& p){
      acc_diff += value;
      acc_diff_sq += value * value;
	  acc.x+= static_cast<float>(p.x);
	  acc.y+= static_cast<float>(p.y);
	  visits++;
	}

	void PointAccumulator::gupdate(double value){
      g_acc_diff += value;
	  g_visits++;
	}

	typedef Map< PointAccumulator, HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;

};

#endif
