#ifndef HARRAY3D_H
#define HARRAY3D_H
#include <set>
#include <../utils/point.h>
#include <../utils/autoptr.h>
#include "array3d.h"

namespace ThreeDOGMapping {

template <class Cell>
class HierarchicalArray3D: public Array3D<autoptr< Array3D<Cell> > >{
	public:
		typedef std::set< point<int>, pointcomparator<int> > PointSet;
		HierarchicalArray3D(int xsize, int ysize, int zsize, int patchMagnitude=1);
		HierarchicalArray3D(const HierarchicalArray3D& hg);
		HierarchicalArray3D& operator=(const HierarchicalArray3D& hg);
		virtual ~HierarchicalArray3D(){}
		void resize(int xmin, int ymin, int zmin, int xmax, int ymax, int zmax);
		inline int getPatchSize() const {return m_patchMagnitude;}
		inline int getPatchMagnitude() const {return m_patchMagnitude;}
		
		inline const Cell& cell(int x, int y, int z) const;
		inline Cell& cell(int x, int y, int z);
		inline bool isAllocated(int x, int y, int z) const;
		inline AccessibilityState cellState(int x, int y, int z) const;
		inline IntPoint patchIndexes(int x, int y, int z) const;
		
		inline const Cell& cell(const IntPoint& p) const { return cell(p.x,p.y,p.z); }
		inline Cell& cell(const IntPoint& p) { return cell(p.x,p.y,p.z); }
		inline bool isAllocated(const IntPoint& p) const { return isAllocated(p.x,p.y,p.z);}
		inline AccessibilityState cellState(const IntPoint& p) const { return cellState(p.x,p.y,p.z); }
		inline IntPoint patchIndexes(const IntPoint& p) const { return patchIndexes(p.x,p.y,p.z);}
		
		inline void setActiveArea(const PointSet&, bool patchCoords=false);
		const PointSet& getActiveArea() const {return m_activeArea; }
		inline void allocActiveArea();
	protected:
		virtual Array3D<Cell> * createPatch(const IntPoint& p) const;
		PointSet m_activeArea;
		int m_patchMagnitude;
		int m_patchSize;
};

template <class Cell>
HierarchicalArray3D<Cell>::HierarchicalArray3D(int xsize, int ysize, int zsize, int patchMagnitude) 
  :Array3D<autoptr< Array3D<Cell> > >::Array3D((xsize>>patchMagnitude), (ysize>>patchMagnitude), (zsize>>patchMagnitude)){
	m_patchMagnitude=patchMagnitude;
	m_patchSize=1<<m_patchMagnitude;
  // std::cout << xsize * ysize * zsize * 4 / 1073741824.0 << std::endl;
}

template <class Cell>
HierarchicalArray3D<Cell>::HierarchicalArray3D(const HierarchicalArray3D& hg)
  :Array3D<autoptr< Array3D<Cell> > >::Array3D((hg.m_xsize>>hg.m_patchMagnitude), (hg.m_ysize>>hg.m_patchMagnitude), (hg.m_zsize>>hg.m_patchMagnitude))
{
	this->m_xsize=hg.m_xsize;
	this->m_ysize=hg.m_ysize;
	this->m_zsize=hg.m_zsize;
	this->m_cells=new autoptr< Array3D<Cell> >**[this->m_xsize];
	for (int x=0; x<this->m_xsize; x++){
		this->m_cells[x]=new autoptr< Array3D<Cell> >*[this->m_ysize];
		for (int y=0; y<this->m_ysize; y++){
      this->m_cells[x][y]=new autoptr< Array3D<Cell> >[this->m_zsize];
			for (int z=0; z<this->m_zsize; z++)
				this->m_cells[x][y][z]=hg.m_cells[x][y][z];
		}
	}
	this->m_patchMagnitude=hg.m_patchMagnitude;
	this->m_patchSize=hg.m_patchSize;
}

template <class Cell>
void HierarchicalArray3D<Cell>::resize(int xmin, int ymin, int zmin, int xmax, int ymax, int zmax){
	int xsize=xmax-xmin;
	int ysize=ymax-ymin;
	int zsize=zmax-zmin;
  // std::cout << xsize << "\n" << ysize << "\n" << zsize << std::endl;
  // std::cout << xsize * ysize * zsize * 4 / 1073741824.0 << std::endl;
	autoptr< Array3D<Cell> >*** newcells=new autoptr< Array3D<Cell> >**[xsize];
	for (int x=0; x<xsize; x++){
		newcells[x]=new autoptr< Array3D<Cell> > *[ysize];
		for (int y=0; y<ysize; y++){
			newcells[x][y]=new autoptr< Array3D<Cell> > [zsize];
    }
	}
	for (int x=0; x<xsize; x++){
		for (int y=0; y<ysize; y++){
			for (int z=0; z<zsize; z++)
				newcells[x][y][z]=autoptr< Array3D<Cell> >(0);
		}
	}
	int dx= xmin < 0 ? 0 : xmin;
	int dy= ymin < 0 ? 0 : ymin;
	int dz= zmin < 0 ? 0 : zmin;
	int Dx=xmax<this->m_xsize?xmax:this->m_xsize;
	int Dy=ymax<this->m_ysize?ymax:this->m_ysize;
	int Dz=zmax<this->m_zsize?zmax:this->m_zsize;
	for (int x=dx; x<Dx; x++) {
		for (int y=dy; y<Dy; y++) {
			for (int z=dz; z<Dz; z++)
				newcells[x-xmin][y-ymin][z-zmin]=this->m_cells[x][y][z];
      delete [] this->m_cells[x][y];
    }
  	delete [] this->m_cells[x];
  }
	delete [] this->m_cells;
	this->m_cells=newcells;
	this->m_xsize=xsize;
	this->m_ysize=ysize;
	this->m_zsize=zsize;
}

template <class Cell>
HierarchicalArray3D<Cell>& HierarchicalArray3D<Cell>::operator=(const HierarchicalArray3D& hg){
	if (this->m_xsize!=hg.m_xsize || this->m_ysize!=hg.m_ysize || this->m_zsize!=hg.m_zsize){
    for (int i=0; i<this->m_xsize; i++)
      for (int j=0; j<this->m_ysize; j++)
        delete [] this->m_cells[i][j];
		for (int i=0; i<this->m_xsize; i++)
			delete [] this->m_cells[i];
		delete [] this->m_cells;
		this->m_xsize=hg.m_xsize;
		this->m_ysize=hg.m_ysize;
		this->m_zsize=hg.m_zsize;
		this->m_cells=new autoptr< Array3D<Cell> >**[this->m_xsize];
		for (int x=0; x<this->m_xsize; x++) {
			this->m_cells[x]=new autoptr< Array3D<Cell> >*[this->m_ysize];
			for (int y=0; y<this->m_ysize; y++)
				this->m_cells[x][y]=new autoptr< Array3D<Cell> > [this->m_zsize];
    }
	}
	for (int x=0; x<this->m_xsize; x++)
		for (int y=0; y<this->m_ysize; y++)
			for (int z=0; z<this->m_zsize; z++)
				this->m_cells[x][y][z]=hg.m_cells[x][y][z];
	
	m_activeArea.clear();
	m_patchMagnitude=hg.m_patchMagnitude;
	m_patchSize=hg.m_patchSize;
	return *this;
}


template <class Cell>
void HierarchicalArray3D<Cell>::setActiveArea(const typename HierarchicalArray3D<Cell>::PointSet& aa, bool patchCoords){
	m_activeArea.clear();
	for (PointSet::const_iterator it= aa.begin(); it!=aa.end(); it++){
		IntPoint p;
		if (patchCoords)
			p=*it;
		else
			p=patchIndexes(*it);
		m_activeArea.insert(p);
	}
}

template <class Cell>
Array3D<Cell>* HierarchicalArray3D<Cell>::createPatch(const IntPoint& ) const{
	return new Array3D<Cell>(1<<m_patchMagnitude, 1<<m_patchMagnitude, 1<<m_patchMagnitude);
}


template <class Cell>
AccessibilityState HierarchicalArray3D<Cell>::cellState(int x, int y, int z) const {
	if (this->isInside(patchIndexes(x,y,z)))
		if(isAllocated(x,y,z))
			return (AccessibilityState)((int)Inside|(int)Allocated);
		else
			return Inside;
	return Outside;
}

template <class Cell>
void HierarchicalArray3D<Cell>::allocActiveArea(){
	for (PointSet::const_iterator it= m_activeArea.begin(); it!=m_activeArea.end(); it++){
		const autoptr< Array3D<Cell> >& ptr=this->m_cells[it->x][it->y][it->z];
		Array3D<Cell>* patch=0;
		if (!ptr){
			patch=createPatch(*it);
		} else {	
			patch=new Array3D<Cell>(*ptr);
		}
		this->m_cells[it->x][it->y][it->z]=autoptr< Array3D<Cell> >(patch);
	}
}

template <class Cell>
bool HierarchicalArray3D<Cell>::isAllocated(int x, int y, int z) const{
	IntPoint c=patchIndexes(x,y,z);
  autoptr< Array3D<Cell> >& ptr=this->m_cells[c.x][c.y][c.z];
	return (ptr != 0);
}

template <class Cell>
IntPoint HierarchicalArray3D<Cell>::patchIndexes(int x, int y, int z) const{
	if (x>=0 && y>=0 && z>= 0)
		return IntPoint(x>>m_patchMagnitude, y>>m_patchMagnitude, z>>m_patchMagnitude);
	return IntPoint(-1, -1, -1);
}

template <class Cell>
Cell& HierarchicalArray3D<Cell>::cell(int x, int y, int z){
	IntPoint c=patchIndexes(x,y,z);
	assert(this->isInside(c.x, c.y, c.z));
	if (!this->m_cells[c.x][c.y][c.z]){
		Array3D<Cell>* patch=createPatch(IntPoint(x,y,z));
		this->m_cells[c.x][c.y][c.z]=autoptr< Array3D<Cell> >(patch);
	}
	autoptr< Array3D<Cell> >& ptr=this->m_cells[c.x][c.y][c.z];
	return (*ptr).cell(IntPoint(x-(c.x<<m_patchMagnitude),y-(c.y<<m_patchMagnitude),z-(c.z<<m_patchMagnitude)));
}

template <class Cell>
const Cell& HierarchicalArray3D<Cell>::cell(int x, int y, int z) const{
	assert(isAllocated(x,y,z));
	IntPoint c=patchIndexes(x,y,z);
	const autoptr< Array3D<Cell> >& ptr=this->m_cells[c.x][c.y][c.z];
	return (*ptr).cell(IntPoint(x-(c.x<<m_patchMagnitude),y-(c.y<<m_patchMagnitude),z-(c.z<<m_patchMagnitude)));
}

};

#endif
