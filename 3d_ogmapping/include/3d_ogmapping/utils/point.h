#ifndef _POINT_H_
#define	_POINT_H_
#include <assert.h>
#include <math.h>
#include <iostream>
#include "gvalues.h"

#define DEBUG_STREAM cerr << __PRETTY_FUNCTION__ << ":" 

namespace ThreeDOGMapping {

	template <class T>
		struct point{
			inline point():x(0),y(0),z(0) {}
			inline point(T _x, T _y, T _z):x(_x),y(_y),z(_z){}
			T x, y, z;
		};

	template <class T>
		inline point<T> operator+(const point<T>& p1, const point<T>& p2){
			return point<T>(p1.x+p2.x, p1.y+p2.y, p1.z+p2.z);
		}

	template <class T>
		inline point<T> operator-(const point<T>& p1, const point<T>& p2) {
			return point<T>(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z);
		}

	template <class T>
		inline point<T> operator*(const point<T>& p, const T& v) {
			return point<T>(p.x*v, p.y*v, p.z*v);
		}
	
	template <class T>
		inline point<T> operator*(const T& v, const point<T>& p) {
			return point<T>(p.x*v, p.y*v, p.z*v);
		}

	template <class T>
		inline T operator*(const point<T>& p1, const point<T>& p2){
			return p1.x*p2.x+p1.y*p2.y+p1.z*p2.z;
		}


	template <class A>
		struct rpy{
			inline rpy():roll(0),pitch(0),yaw(0) {}
			inline rpy(A _roll, A _pitch, A _yaw):roll(_roll),pitch(_pitch),yaw(_yaw){}
			A roll, pitch, yaw;
		};

	template <class A>
		inline rpy<A> operator+(const rpy<A>& r1, const rpy<A>& r2){
			return rpy<A>(r1.roll+r2.roll, r1.pitch+r2.pitch, r1.yaw+r2.yaw);
		}

	template <class A>
		inline rpy<A> operator-(const rpy<A>& r1, const rpy<A>& r2) {
			return rpy<A>(r1.roll-r2.roll, r1.pitch-r2.pitch, r1.yaw-r2.yaw);
		}

	template <class A>
		inline rpy<A> operator*(const rpy<A>& r, const A& v) {
			return rpy<A>(r.roll*v, r.pitch*v, r.yaw*v);
		}
	
	template <class A>
		inline rpy<A> operator*(const A& v, const rpy<A>& r) {
		return rpy<A>(r.roll*v, r.pitch*v, r.yaw*v);
		}

	template <class T, class A>
		struct pose:  public point<T>, rpy<A>{
			inline pose() : point<T>(0,0,0), rpy<A>(0, 0, 0) {};
			inline pose(T x, T y, T z, A roll, A pitch, A yaw): point<T>(x,y,z), rpy<A>(roll,pitch,yaw){}
			inline pose(const point<T>& p,const rpy<A>& r);
		};

	template <class T, class A>
		pose<T,A>::pose(const point<T>& p, const rpy<A>& r){
			this->x=p.x;
			this->y=p.y;
			this->z=p.z;
			this->roll=r.roll;
			this->pitch=r.pitch;
			this->yaw=r.yaw;
		}

	template <class T, class A>
		pose<T,A> operator + (const pose<T,A>& p1, const pose<T,A>& p2){
			return pose<T,A>(p1.x+p2.x, p1.y+p2.y, p1.z+p2.z, p1.roll+p2.roll, p1.pitch+p2.pitch, p1.yaw+p2.yaw);
		}
	template <class T, class A>
		pose<T,A> operator - (const pose<T,A> & p1, const pose<T,A> & p2){
			return pose<T,A>(p1.x-p2.x, p1.y-p2.y, p1.z-p2.z, p1.roll-p2.roll, p1.pitch-p2.pitch, p1.yaw-p2.yaw);
		}

	template <class T, class A>
		pose<T,A> operator * (const pose<T,A>& p, const T& v){
			return pose<T,A>(p.x*v, p.y*v, p.z*v, p.roll*v, p.pitch*v, p.yaw*v);
		}

	template <class T, class A>
		pose<T,A> operator * (const T& v, const pose<T,A>& p){
			return pose<T,A>(p.x*v, p.y*v, p.z*v, p.roll*v, p.pitch*v, p.yaw*v);
		}

	template <class T>
		struct pointcomparator{
			bool operator ()(const point<T>& a, const point<T>& b) const {
				return a.x<b.x || (a.x==b.x && a.y<b.y) || (a.x==b.x && a.y==b.y && a.z<b.z);
			}
		};

	template <class T>
		inline point<T> max(const point<T>& p1, const point<T>& p2){
			point<T> p=p1;
			p.x=p.x>p2.x?p.x:p2.x;
			p.y=p.y>p2.y?p.y:p2.y;
			p.z=p.z>p2.z?p.z:p2.z;
			return p;
		}
	template <class T>
		inline point<T> min(const point<T>& p1, const point<T>& p2){
			point<T> p=p1;
			p.x=p.x<p2.x?p.x:p2.x;
			p.y=p.y<p2.y?p.y:p2.y;
			p.z=p.z<p2.z?p.z:p2.z;
			return p;
		}

typedef point<int> IntPoint;
typedef point<double> Point;
typedef rpy<double> RPY;
typedef pose<double, double> Pose;

}; //end namespace


#endif
