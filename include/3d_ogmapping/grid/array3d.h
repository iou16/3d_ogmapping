#ifndef ARRAY3D_H
#define ARRAY3D_H

#include <assert.h>
#include <../utils/point.h>
#include "accessstate.h"

#include <iostream>

namespace ThreeDOGMapping {

	template<class Cell, const bool debug=false> class Array3D{
		public:
			Array3D(int xsize=0, int ysize=0, int zsize=0);
			Array3D& operator=(const Array3D &);
			Array3D(const Array3D<Cell,debug> &);
			~Array3D();
			void clear();
			void resize(int xmin, int ymin, int zmin, int xmax, int ymax, int zmax);

			inline bool isInside(int x, int y, int z) const;
			inline const Cell& cell(int x, int y, int z) const;
			inline Cell& cell(int x, int y, int z);
			inline AccessibilityState cellState(int x, int y, int z) const {return (AccessibilityState) (isInside(x,y,z)?(Inside|Allocated):Outside);}

			inline bool isInside(const IntPoint& p) const {return isInside(p.x, p.y, p.z);}
			inline const Cell& cell(const IntPoint& p) const {return cell(p.x,p.y,p.z);}
			inline Cell& cell(const IntPoint& p) {return cell(p.x,p.y,p.z);}
			inline AccessibilityState cellState(const IntPoint& p) const {return cellState(p.x,p.y,p.z);}

			inline int getPatchSize() const {return 0;}
			inline int getPatchMagnitude() const {return 0;}
			inline int getXSize() const {return m_xsize;}
			inline int getYSize() const {return m_ysize;}
			inline int getZSize() const {return m_zsize;}
			inline Cell*** cells() {return m_cells;}
			Cell *** m_cells;
		protected:
			int m_xsize, m_ysize, m_zsize;
	};

	template <class Cell, const bool debug>
		Array3D<Cell,debug>::Array3D(int xsize, int ysize, int zsize){
			m_xsize=xsize;
			m_ysize=ysize;
			m_zsize=zsize;
			if (m_xsize>0 && m_ysize>0 && m_zsize>0){
				m_cells=new Cell**[m_xsize];
				for(int i=0; i<m_xsize; i++) {
				  m_cells[i]=new Cell*[m_ysize];
					for(int j=0; j<m_ysize; j++) 
						m_cells[i][j]=new Cell[m_zsize];
        }
			} else {
				m_xsize=m_ysize=m_zsize=0;
				m_cells=0;
			}
		}

	template <class Cell, const bool debug>
		Array3D<Cell,debug> & Array3D<Cell,debug>::operator=(const Array3D<Cell,debug>& g){
			if (m_xsize!=g.m_xsize || m_ysize!=g.m_ysize || m_zsize!=g.m_zsize){
				for (int i=0; i<m_xsize; i++)
					for (int j=0; j<m_ysize; j++)
						delete [] m_cells[i][j];
				for (int i=0; i<m_xsize; i++)
					delete [] m_cells[i];
				delete [] m_cells;
				m_xsize=g.m_xsize;
				m_ysize=g.m_ysize;
        m_zsize=g.m_zsize;
				m_cells=new Cell**[m_xsize];
				for (int i=0; i<m_xsize; i++) {
					m_cells[i]=new Cell*[m_ysize];
					for(int j=0; j<m_ysize; j++)
						m_cells[i][j]=new Cell[m_zsize];
        }
			}
			for (int x=0; x<m_xsize; x++)
				for (int y=0; y<m_ysize; y++)
					for (int z=0; z<m_zsize; z++)
						m_cells[x][y][z]=g.m_cells[x][y][z];
      return *this;
		}

  template <class Cell, const bool debug>
    Array3D<Cell,debug>::Array3D(const Array3D<Cell,debug> & g){
    	m_xsize=g.m_xsize;
    	m_ysize=g.m_ysize;
      m_zsize=g.m_zsize;
    	m_cells=new Cell**[m_xsize];
    	for (int x=0; x<m_xsize; x++) {
					m_cells[x]=new Cell*[m_ysize];
				  for(int y=0; y<m_ysize; y++) {
				  	m_cells[x][y]=new Cell[m_zsize];
   				  for (int z=0; z<m_zsize; z++)
   				  	m_cells[x][y][z]=g.m_cells[x][y][z];
          }
      }
    }

	template <class Cell, const bool debug>
		Array3D<Cell,debug>::~Array3D(){
			for (int i=0; i<m_xsize; i++) {
					for (int j=0; j<m_ysize; j++) {
						delete [] m_cells[i][j];
            m_cells[i][j]=0;
          }
      }
			for (int i=0; i<m_xsize; i++) {
				delete [] m_cells[i];
        m_cells[i]=0;
      }
			delete [] m_cells;
			m_cells=0;
		}

	template <class Cell, const bool debug>
		void Array3D<Cell,debug>::clear(){
			for (int i=0; i<m_xsize; i++) {
				for (int j=0; j<m_ysize; j++) {
					delete [] m_cells[i][j];
          m_cells[i][j]=0;
        }
      }
			for (int i=0; i<m_xsize; i++) {
				delete [] m_cells[i];
        m_cells[i]=0;
      }
			delete [] m_cells;
			m_cells=0;
			m_xsize=0;
			m_ysize=0;
			m_zsize=0;
		}

	template <class Cell, const bool debug>
		void Array3D<Cell,debug>::resize(int xmin, int ymin, int zmin, int xmax, int ymax, int zmax){
			int xsize=xmax-xmin;
			int ysize=ymax-ymin;
			int zsize=zmax-zmin;
			Cell *** newcells=new Cell **[xsize];
			for (int i=0; i<m_xsize; i++) {
				newcells[i]=new Cell*[m_ysize];
				for(int j=0; j<m_ysize; j++) 
					newcells[i][j]=new Cell[m_zsize];
      }
			int dx= xmin < 0 ? 0 : xmin;
			int dy= ymin < 0 ? 0 : ymin;
			int dz= zmin < 0 ? 0 : zmin;
			int Dx=xmax<this->m_xsize?xmax:this->m_xsize;
			int Dy=ymax<this->m_ysize?ymax:this->m_ysize;
			int Dz=zmax<this->m_zsize?zmax:this->m_zsize;
			for (int x=dx; x<Dx; x++){
				for (int y=dy; y<Dy; y++){
					for (int z=dz; z<Dz; z++){
					newcells[x-xmin][y-ymin][z-zmin]=this->m_cells[x][y][z];
					}
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

	template <class Cell, const bool debug>
		inline bool Array3D<Cell,debug>::isInside(int x, int y, int z) const{
			return x>=0 && y>=0 && z>=0 && x<m_xsize && y<m_ysize && z<m_zsize; 
		}
	
	template <class Cell, const bool debug>
		inline const Cell& Array3D<Cell,debug>::cell(int x, int y, int z) const{
			assert(isInside(x,y,z));
			return m_cells[x][y][z];
		}

	template <class Cell, const bool debug>
		inline Cell& Array3D<Cell,debug>::cell(int x, int y, int z) {
			assert(isInside(x,y,z));
			return m_cells[x][y][z];
		}

};

#endif
