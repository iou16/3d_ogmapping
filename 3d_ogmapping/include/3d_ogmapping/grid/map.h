#ifndef MAP_H
#define MAP_H
#include <../utils/point.h>
#include <assert.h>
#include "accessstate.h"
#include "array3d.h"

namespace ThreeDOGMapping {

template <class Cell, class Storage, const bool isClass=true> 
class Map{
	public:
		Map(int mapSizeX, int mapSizeY, int mapSizeZ, double delta);
		Map(const Point& center, double worldSizeX, double worldSizeY, double worldSizeZ, double delta);
		Map(const Point& center, double xmin, double ymin, double zmin, double xmax, double ymax, double zmax, double delta);
		void resize(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax);
		void grow(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax);
		inline IntPoint world2map(const Point& p) const;
		inline Point map2world(const IntPoint& p) const;
		inline IntPoint world2map(double x, double y, double z) const 
		  { return world2map(Point(x,y,z)); }
		inline Point map2world(int x, int y, int z) const 
		  { return map2world(IntPoint(x,y,z)); }

		inline Point getCenter() const {return m_center;}	
		inline double getWorldSizeX() const {return m_worldSizeX;}
		inline double getWorldSizeY() const {return m_worldSizeY;}
    inline double getWorldSizeZ() const {return m_worldSizeZ;}
		inline int getMapSizeX() const {return m_mapSizeX;}
		inline int getMapSizeY() const {return m_mapSizeY;}
    inline int getMapSizeZ() const {return m_mapSizeZ;}
		inline double getDelta() const { return m_delta;}
		inline double getMapResolution() const { return m_delta;}
		inline double getResolution() const { return m_delta;}
		inline void getSize(double & xmin, double& ymin, double zmin, double& xmax, double& ymax, double& zmax) const {
			Point min=map2world(0,0,0), max=map2world(IntPoint(m_mapSizeX-1, m_mapSizeY-1, m_mapSizeZ-1)); 
			xmin=min.x, ymin=min.y, zmin=min.z, xmax=max.x, ymax=max.y, zmax=max.z; 
		}
		
		inline Cell& cell(int x, int y, int z) {
		  return cell(IntPoint(x, y, z));
		}
		inline Cell& cell(const IntPoint& p);

		inline const Cell& cell(int x, int y, int z) const  {
		  return cell_(IntPoint(x, y, z));
		}
		inline const Cell& cell_(const IntPoint& p) const;

		inline Cell& cell(double x, double y, double z) {
		  return cell(Point(x, y, z));
		}
		inline Cell& cell(const Point& p);

		inline const Cell& cell(double x, double y, double z) const {
		  return cell(Point(x, y, z));
		}

		inline bool isInside(int x, int y, int z) const {
		  return m_storage.cellState(IntPoint(x,y,z))&Inside;
		}

		inline bool isInside(const IntPoint& p) const {
		  return m_storage.cellState(p)&Inside;
		}

		inline bool isInside(double x, double y, double z) const {
		  return m_storage.cellState(world2map(x,y,z))&Inside;
		}
		inline bool isInside(const Point& p) const {
		  return m_storage.cellState(world2map(p))&Inside;
		}

		inline const Cell& cell(const Point& p) const;

		inline Storage& storage() { return m_storage; }
		inline const Storage& storage() const { return m_storage; }
		
	protected:
		Point m_center;
		double m_worldSizeX, m_worldSizeY, m_worldSizeZ, m_delta;
		Storage m_storage;
		int m_mapSizeX, m_mapSizeY, m_mapSizeZ;
		int m_sizeX2, m_sizeY2, m_sizeZ2;
	static const Cell m_unknown;
};


template <class Cell, class Storage, const bool isClass>
  const Cell  Map<Cell,Storage,isClass>::m_unknown = Cell(-1);

template <class Cell, class Storage, const bool isClass>
Map<Cell,Storage,isClass>::Map(int mapSizeX, int mapSizeY, int mapSizeZ, double delta):
	m_storage(mapSizeX, mapSizeY, mapSizeZ){
	m_worldSizeX=mapSizeX * delta;
	m_worldSizeY=mapSizeY * delta;
	m_worldSizeZ=mapSizeZ * delta;
	m_delta=delta;
	m_center=Point(0.5*m_worldSizeX, 0.5*m_worldSizeY, 0.5*m_worldSizeZ);
	m_sizeX2=m_mapSizeX>>1;
	m_sizeY2=m_mapSizeY>>1;
	m_sizeZ2=m_mapSizeZ>>1;
}

template <class Cell, class Storage, const bool isClass>
Map<Cell,Storage,isClass>::Map(const Point& center, double worldSizeX, double worldSizeY, double worldSizeZ, double delta):
	m_storage((int)ceil(worldSizeX/delta), (int)ceil(worldSizeY/delta), (int)ceil(worldSizeZ/delta)){
	m_center=center;
	m_worldSizeX=worldSizeX;
	m_worldSizeY=worldSizeY;
	m_worldSizeZ=worldSizeZ;
	m_delta=delta;
	m_mapSizeX=m_storage.getXSize()<<m_storage.getPatchSize();
	m_mapSizeY=m_storage.getYSize()<<m_storage.getPatchSize();
	m_mapSizeZ=m_storage.getZSize()<<m_storage.getPatchSize();
	m_sizeX2=m_mapSizeX>>1;
	m_sizeY2=m_mapSizeY>>1;
	m_sizeZ2=m_mapSizeZ>>1;
}

template <class Cell, class Storage, const bool isClass>
Map<Cell,Storage,isClass>::Map(const Point& center, double xmin, double ymin, double zmin, double xmax, double ymax, double zmax, double delta):
	m_storage((int)ceil((xmax-xmin)/delta), (int)ceil((ymax-ymin)/delta), (int)ceil((zmax-zmin)/delta)){
	m_center=center;
	m_worldSizeX=xmax-xmin;
	m_worldSizeY=ymax-ymin;
	m_worldSizeZ=zmax-zmin;
	m_delta=delta;
	m_mapSizeX=m_storage.getXSize()<<m_storage.getPatchSize();
	m_mapSizeY=m_storage.getYSize()<<m_storage.getPatchSize();
	m_mapSizeZ=m_storage.getZSize()<<m_storage.getPatchSize();
	m_sizeX2=(int)round((m_center.x-xmin)/m_delta);
	m_sizeY2=(int)round((m_center.y-ymin)/m_delta);
	m_sizeZ2=(int)round((m_center.z-zmin)/m_delta);
}
		
template <class Cell, class Storage, const bool isClass>
void Map<Cell,Storage,isClass>::resize(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax){
	IntPoint imin=world2map(xmin, ymin, zmin);
	IntPoint imax=world2map(xmax, ymax, zmax);
	int pxmin, pymin, pzmin, pxmax, pymax, pzmax;
	pxmin=(int)floor((float)imin.x/(1<<m_storage.getPatchMagnitude()));
	pymin=(int)floor((float)imin.y/(1<<m_storage.getPatchMagnitude()));
	pzmin=(int)floor((float)imin.z/(1<<m_storage.getPatchMagnitude()));
	pxmax=(int)ceil((float)imax.x/(1<<m_storage.getPatchMagnitude()));
	pymax=(int)ceil((float)imax.y/(1<<m_storage.getPatchMagnitude()));
	pzmax=(int)ceil((float)imax.z/(1<<m_storage.getPatchMagnitude()));
	m_storage.resize(pxmin, pymin, pzmin, pxmax, pymax, pzmax);
	m_mapSizeX=m_storage.getXSize()<<m_storage.getPatchSize();
	m_mapSizeY=m_storage.getYSize()<<m_storage.getPatchSize();
	m_mapSizeZ=m_storage.getZSize()<<m_storage.getPatchSize();
	m_worldSizeX=xmax-xmin;
	m_worldSizeY=ymax-ymin;
	m_worldSizeZ=zmax-zmin;
	m_sizeX2-=pxmin*(1<<m_storage.getPatchMagnitude());
	m_sizeY2-=pymin*(1<<m_storage.getPatchMagnitude());
	m_sizeZ2-=pzmin*(1<<m_storage.getPatchMagnitude());
}

template <class Cell, class Storage, const bool isClass>
void Map<Cell,Storage,isClass>::grow(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax){
	IntPoint imin=world2map(xmin, ymin, zmin);
	IntPoint imax=world2map(xmax, ymax, zmax);
	if (isInside(imin) && isInside(imax))
		return;
	imin=min(imin, IntPoint(0,0,0));
	imax=max(imax, IntPoint(m_mapSizeX-1,m_mapSizeY-1,m_mapSizeZ-1));
	int pxmin, pymin, pzmin, pxmax, pymax, pzmax;
	pxmin=(int)floor((float)imin.x/(1<<m_storage.getPatchMagnitude()));
	pxmax=(int)ceil((float)imax.x/(1<<m_storage.getPatchMagnitude()));
	pymin=(int)floor((float)imin.y/(1<<m_storage.getPatchMagnitude()));
	pymax=(int)ceil((float)imax.y/(1<<m_storage.getPatchMagnitude()));
	pzmin=(int)floor((float)imin.z/(1<<m_storage.getPatchMagnitude()));
	pzmax=(int)ceil((float)imax.z/(1<<m_storage.getPatchMagnitude()));
	m_storage.resize(pxmin, pymin, pzmin, pxmax, pymax, pzmax);
	m_mapSizeX=m_storage.getXSize()<<m_storage.getPatchSize();
	m_mapSizeY=m_storage.getYSize()<<m_storage.getPatchSize();
	m_mapSizeZ=m_storage.getZSize()<<m_storage.getPatchSize();
	m_worldSizeX=xmax-xmin;
	m_worldSizeY=ymax-ymin;
	m_worldSizeZ=zmax-zmin;
	m_sizeX2-=pxmin*(1<<m_storage.getPatchMagnitude());
	m_sizeY2-=pymin*(1<<m_storage.getPatchMagnitude());
	m_sizeZ2-=pzmin*(1<<m_storage.getPatchMagnitude());
}


template <class Cell, class Storage, const bool isClass>
IntPoint Map<Cell,Storage,isClass>::world2map(const Point& p) const{
	return IntPoint( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2, (int)round((p.z-m_center.z)/m_delta)+m_sizeZ2);
}

template <class Cell, class Storage, const bool isClass>
Point Map<Cell,Storage,isClass>::map2world(const IntPoint& p) const{
	return Point( (p.x-m_sizeX2)*m_delta,
								(p.y-m_sizeY2)*m_delta, 
								(p.z-m_sizeZ2)*m_delta )+m_center;
}

template <class Cell, class Storage, const bool isClass>
Cell& Map<Cell,Storage,isClass>::cell(const IntPoint& p) {
	AccessibilityState s=m_storage.cellState(p);
	if (! s&Inside)
		assert(0);

	return m_storage.cell(p);

}

template <class Cell, class Storage, const bool isClass>
Cell& Map<Cell,Storage,isClass>::cell(const Point& p) {
	IntPoint ip=world2map(p);
	AccessibilityState s=m_storage.cellState(ip);
	if (! s&Inside)
		assert(0);

	return m_storage.cell(ip);
}

template <class Cell, class Storage, const bool isClass>
  const Cell& Map<Cell,Storage,isClass>::cell_(const IntPoint& p) const {
  AccessibilityState s=m_storage.cellState(p);
  if (s&Allocated)	 
    return m_storage.cell(p);
  return m_unknown;
}

template <class Cell, class Storage, const bool isClass>
const  Cell& Map<Cell,Storage,isClass>::cell(const Point& p) const {
  IntPoint ip=world2map(p);
  AccessibilityState s=m_storage.cellState(ip);
  if (s&Allocated)	 
    return m_storage.cell(ip);
  return  m_unknown;
}

};

#endif

