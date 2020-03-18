/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                 CONTRAST MAXIMISATION BRANCH AND BOUND
%
%
% This package contains the source code which implements the
% Contrast maximisation BnB algorithm (CMBnB) in
%
%       Globally Optimal Contrast Maximisation for Event-based  
%                       Motion Estimation
%
% The source code, binaries and demo are supplied for academic use only.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#ifndef REG_STATE_
#define REG_STATE_

#include <iostream>
#include "reg_common.h"

namespace reg {
    namespace search {

        //----------------------------------------------------
        //       Search Space Regions (SSR)
        //----------------------------------------------------

        struct RotationSearchSpaceRegion1DOF
        {
            double lw; // Lower angle
            double up; // Upper angle

            RotationSearchSpaceRegion1DOF() {lw=0;up=TWOPI;}
            RotationSearchSpaceRegion1DOF(double lw, double up):
            lw(lw), up(up) {}
        };

        inline
        std::ostream &operator<<(std::ostream& os, const RotationSearchSpaceRegion1DOF &reg)
        {
            os << "["<< reg.lw <<" "<< reg.up <<"]";
            return os;
        }


        struct TranslationSearchSpaceRegion3DOF
        {
            double ax, ay, az;
            double bx, by, bz;

            TranslationSearchSpaceRegion3DOF() {}
            TranslationSearchSpaceRegion3DOF(double ax, double ay, double az,
                                             double bx, double by, double bz):
            ax(ax), ay(ay), az(az),
            bx(bx), by(by), bz(bz) {}

            TranslationSearchSpaceRegion3DOF(double *x):
            ax(x[0]), ay(x[1]), az(x[2]),
            bx(x[3]), by(x[4]), bz(x[5]) {}
        };


        inline
        std::ostream &operator<<(std::ostream& os, const TranslationSearchSpaceRegion3DOF &reg)
        {
            os << "[("<< reg.ax <<" "<< reg.ay <<" "<< reg.az <<")"
            << " ("<< reg.bx <<" "<< reg.by <<" "<< reg.bz <<")]";
            return os;
        }

        struct RotationSearchSpaceRegion3DOFS2
        {
            double ax, ay, az;
            double bx, by, bz;

            RotationSearchSpaceRegion3DOFS2():
            ax(-PI), ay(-PI), az(-PI),
            bx(PI), by(PI), bz(PI) {}

            RotationSearchSpaceRegion3DOFS2(double ax, double ay, double az,
                                            double bx, double by, double bz):
            ax(ax), ay(ay), az(az),
            bx(bx), by(by), bz(bz) {}
        };

        inline
        std::ostream &operator<<(std::ostream& os, const RotationSearchSpaceRegion3DOFS2 &reg)
        {
            os << "[("<< reg.ax <<" "<< reg.ay <<" "<< reg.az <<")"
            << " ("<< reg.bx <<" "<< reg.by <<" "<< reg.bz <<")]";
            return os;
        }

        typedef
        struct RotationSearchSpaceRegion3DOFS8
        {
            double ax, ay, az;
            double bx, by, bz;

            RotationSearchSpaceRegion3DOFS8():
            ax(-PI), ay(-PI), az(-PI),
            bx(PI), by(PI), bz(PI) {}

            RotationSearchSpaceRegion3DOFS8(double ax, double ay, double az,
                                            double bx, double by, double bz):
            ax(ax), ay(ay), az(az),
            bx(bx), by(by), bz(bz) {}
        } RotationSearchSpaceRegion3DOFS8;

        inline
        std::ostream &operator<<(std::ostream& os, const RotationSearchSpaceRegion3DOFS8 &reg)
        {
            os << "[("<< reg.ax <<" "<< reg.ay <<" "<< reg.az <<")"
            << " ("<< reg.bx <<" "<< reg.by <<" "<< reg.bz <<")]";
            return os;
        }

        inline
        reg::Vector3 ssrAxis(RotationSearchSpaceRegion3DOFS2 ssr)
        {
            reg::Vector3 axis( ssr.ax + ssr.bx,
                              ssr.ay + ssr.by,
                              ssr.az + ssr.bz );

            const double norm = axis.norm();

            if (norm<DUMMY_PRECISION)
            {
                return  Vector3(1,0,0);
            }
            axis.x /= norm;
            axis.y /= norm;
            axis.z /= norm;

            return axis;
        }


        inline
        double ssrAngle(RotationSearchSpaceRegion3DOFS2 ssr)
        {
            double angle;
            const double dx = ssr.ax + ssr.bx;
            const double dy = ssr.ay + ssr.by;
            const double dz = ssr.az + ssr.bz;

            angle = .5*sqrt(dx*dx + dy*dy + dz*dz);
            return angle>PI ? PI : angle;
        }


        inline
        double ssrMaxAngle(RotationSearchSpaceRegion3DOFS8 ssr)
        {
            // compute max. rotation angle
            const double dx = ssr.bx - ssr.ax;
            mxAssert(dx>=0 &&
                     fabs(dx-(ssr.by-ssr.ay))<=DUMMY_PRECISION &&
                     fabs(dx-(ssr.bz-ssr.az))<=DUMMY_PRECISION, "");

            const double r = .5*dx*sqrt(3.0);

            return r>PI ? PI : r;
        }


        inline
        AxisAngle centre(RotationSearchSpaceRegion3DOFS8 ssr)
        {
            double angle;
            const double dx = ssr.ax + ssr.bx;
            const double dy = ssr.ay + ssr.by;
            const double dz = ssr.az + ssr.bz;

            Vector3 axis(dx, dy,dz);
            const double norm = axis.norm();

            if (norm<DUMMY_PRECISION)
            {
                axis = Vector3(1,0,0);
            }
            else
            {
                axis.x /= norm;
                axis.y /= norm;
                axis.z /= norm;
            }

            angle = .5*sqrt(dx*dx + dy*dy + dz*dz);
            if(angle>PI)
            {
                angle = PI;
            }

            mxAssert(fabs(sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z)-1.0)<DUMMY_PRECISION,
                     "axis must be an unit-vector");


            return AxisAngle(axis,angle);
        }


        inline
        Matrix3 centre(RotationSearchSpaceRegion3DOFS2 ssr)
        {
            double angle;
            const double dx = ssr.ax + ssr.bx;
            const double dy = ssr.ay + ssr.by;
            const double dz = ssr.az + ssr.bz;

            Vector3 axis(dx,dy,dz);
            const double norm = axis.norm();

            if (norm<DUMMY_PRECISION)
            {
                axis = Vector3(1,0,0);
            }
            else
            {
                axis.x /= norm;
                axis.y /= norm;
                axis.z /= norm;
            }

            angle = .5*sqrt(dx*dx + dy*dy + dz*dz);
            if(angle>PI)
            {
                angle = PI;
            }
            //    const Eigen::AngleAxisd angleAxis(angle, axis);
            //    return angleAxis.toRotationMatrix();
            Matrix3 R;
            fromAxisAngle(R,AxisAngle(axis, angle));
            return R;
        }


        inline
        double centre(RotationSearchSpaceRegion1DOF ssr)
        {
            return .5*(ssr.lw+ssr.up);
        }



        inline
        double ssrMaxAngle(RotationSearchSpaceRegion3DOFS2 ssr)
        {
            // Compute max. rotation angle
            const double dx = ssr.bx-ssr.ax;
            const double dy = ssr.by-ssr.ay;
            const double dz = ssr.bz-ssr.az;
            mxAssert(dx>=0 && dy>=0 && dz>=0,"");
            const double r = .5*sqrt(dx*dx + dy*dy + dz*dz);
            return r>PI ? PI :  r;
        }

        inline
        double ssrMaxAngle(RotationSearchSpaceRegion1DOF ssr)
        {
            // Compute max. rotation angle
            double r = .5*(ssr.up-ssr.lw);
            return  r;
        }


        inline
        int split(RotationSearchSpaceRegion1DOF ssr,
                  RotationSearchSpaceRegion1DOF **partition)
        {
            const double centre = .5*(ssr.lw + ssr.up);
            partition[0] = new RotationSearchSpaceRegion1DOF(ssr.lw, centre);
            partition[1] = new RotationSearchSpaceRegion1DOF(centre, ssr.up);

            return 2;
        }


        inline
        int split(TranslationSearchSpaceRegion3DOF ssr,
                  TranslationSearchSpaceRegion3DOF **partition)
        {
            const double ax = ssr.ax;
            const double ay = ssr.ay;
            const double az = ssr.az;

            const double bx = ssr.bx;
            const double by = ssr.by;
            const double bz = ssr.bz;

            if(bx-ax < EPSILON)
            {
                std::cout<<"warning! "<<std::endl;
                return 0;
            }

            const double mx = .5*(ax + bx);
            const double my = .5*(ay + by);
            const double mz = .5*(az + bz);

            partition[0] = new TranslationSearchSpaceRegion3DOF(ax, ay, az, mx, my, mz);
            partition[1] = new TranslationSearchSpaceRegion3DOF(mx, ay, az, bx, my, mz);
            partition[2] = new TranslationSearchSpaceRegion3DOF(ax, my, az, mx, by, mz);
            partition[3] = new TranslationSearchSpaceRegion3DOF(mx, my, az, bx, by, mz);
            partition[4] = new TranslationSearchSpaceRegion3DOF(ax, ay, mz, mx, my, bz);
            partition[5] = new TranslationSearchSpaceRegion3DOF(mx, ay, mz, bx, my, bz);
            partition[6] = new TranslationSearchSpaceRegion3DOF(ax, my, mz, mx, by, bz);
            partition[7] = new TranslationSearchSpaceRegion3DOF(mx, my, mz, bx, by, bz);

            return 8;
        }


        inline
        Vector3 ssrCentre(TranslationSearchSpaceRegion3DOF ssr)
        {
            Vector3 c (.5*(ssr.ax + ssr.bx),
                       .5*(ssr.ay + ssr.by),
                       .5*(ssr.az + ssr.bz));
            return c;
        }

        inline
        double ssrUncertainty(TranslationSearchSpaceRegion3DOF ssr)
        {
            /* compute max. translation uncertainty */
            const double dx = ssr.ax - ssr.bx;
            const double dy = ssr.ay - ssr.by;
            const double dz = ssr.az - ssr.bz;
            return .5*sqrt(dx*dx + dy*dy + dz*dz);
        }


        inline bool
        box_intersects_phi_sphere(double ax, double ay, double az,
                                  double bx, double by, double bz)
        {
            double cx, cy, cz;
            const double pi2 = PI*PI;

            cx = .5*(ax + bx); cx *= cx;
            cy = .5*(ay + by); cy *= cy;
            cz = .5*(az + bz); cz *= cz;

            if (cx + cy + cz <= pi2 ) // centre
            {
                return true;
            }

            ax *= ax; ay *= ay; az *= az;
            if (ax + ay + az <= pi2 ) // a
            {
                return true;
            }

            bx *= bx; by *= by; bz *= bz;

            //check remaining 7 corners
            return  bx + by + bz <= pi2 ||
            ax + by + az <= pi2 ||
            bx + by + az <= pi2 ||
            bx + ay + az <= pi2 ||
            bx + ay + bz <= pi2 ||
            ax + ay + bz <= pi2 ||
            ax + by + bz <= pi2;
        }



        inline
        int split(RotationSearchSpaceRegion3DOFS2 ssr,
                  RotationSearchSpaceRegion3DOFS2 **partition)
        {
            const double ax = ssr.ax;
            const double ay = ssr.ay;
            const double az = ssr.az;

            const double bx = ssr.bx;
            const double by = ssr.by;
            const double bz = ssr.bz;

            if(bx-ax < EPSILON)
            {
                return 0;
            }

            const double mx = .5*(ax + bx);
            const double my = .5*(ay + by);
            const double mz = .5*(az + bz);

            const double dx = mx-ax;
            const double dy = my-ay;
            const double dz = mz-az;

            int np=0;

            mxAssert(dx>=0 , "dx must be positive");
            mxAssert(dx>=0 , "dy must be positive");
            mxAssert(dx>=0 , "dz must be positive");

            if (dx>dy && dx>dz)
            {
                if( box_intersects_phi_sphere                            (ax,ay,az,mx,by,bz))
                    partition[np++] = new RotationSearchSpaceRegion3DOFS2(ax,ay,az,mx,by,bz);

                if(box_intersects_phi_sphere                             (mx,ay,az,bx,by,bz))
                    partition[np++] = new RotationSearchSpaceRegion3DOFS2(mx,ay,az,bx,by,bz);
            }
            else if (dy > dz)
            {
                if (box_intersects_phi_sphere                            (ax,ay,az,bx,my,bz))
                    partition[np++] = new RotationSearchSpaceRegion3DOFS2(ax,ay,az,bx,my,bz);
                if (box_intersects_phi_sphere                            (ax,my,az,bx,by,bz))
                    partition[np++] = new RotationSearchSpaceRegion3DOFS2(ax,my,az,bx,by,bz);
            }
            else
            {
                if (box_intersects_phi_sphere                            (ax,ay,az,bx,by,mz))
                    partition[np++] = new RotationSearchSpaceRegion3DOFS2(ax,ay,az,bx,by,mz);
                if (box_intersects_phi_sphere                            (ax,ay,mz,bx,by,bz))
                    partition[np++] = new RotationSearchSpaceRegion3DOFS2(ax,ay,mz,bx,by,bz);
            }

            return np;
        }


        inline
        int split(RotationSearchSpaceRegion3DOFS8 ssr,
                  RotationSearchSpaceRegion3DOFS8 **partition)
        {
            int np=0;

            const double ax = ssr.ax;
            const double ay = ssr.ay;
            const double az = ssr.az;

            const double bx = ssr.bx;
            const double by = ssr.by;
            const double bz = ssr.bz;

            if(bx-ax < EPSILON)
            {
                return 0;
            }

            mxAssert(bx>=ax && by>=ay && bz>=az,"");

            const double mx = .5*(ax + bx);
            const double my = .5*(ay + by);
            const double mz = .5*(az + bz);

            if(box_intersects_phi_sphere(ax, ay, az, mx, my, mz))
            {
                partition[np++] = new RotationSearchSpaceRegion3DOFS8(ax, ay, az, mx, my, mz);
            }
            if(box_intersects_phi_sphere(mx, ay, az, bx, my, mz))
            {
                partition[np++] = new RotationSearchSpaceRegion3DOFS8(mx, ay, az, bx, my, mz);
            }
            if(box_intersects_phi_sphere(ax, my, az, mx, by, mz))
            {
                partition[np++] = new RotationSearchSpaceRegion3DOFS8(ax, my, az, mx, by, mz);
            }
            if(box_intersects_phi_sphere(mx, my, az, bx, by, mz))
            {
                partition[np++] = new RotationSearchSpaceRegion3DOFS8(mx, my, az, bx, by, mz);
            }
            if(box_intersects_phi_sphere(ax, ay, mz, mx, my, bz))
            {
                partition[np++] = new RotationSearchSpaceRegion3DOFS8(ax, ay, mz, mx, my, bz);
            }
            if(box_intersects_phi_sphere(mx, ay, mz, bx, my, bz))
            {
                partition[np++] = new RotationSearchSpaceRegion3DOFS8(mx, ay, mz, bx, my, bz);
            }
            if(box_intersects_phi_sphere(ax, my, mz, mx, by, bz))
            {
                partition[np++] = new RotationSearchSpaceRegion3DOFS8(ax, my, mz, mx, by, bz);
            }
            if(box_intersects_phi_sphere(mx, my, mz, bx, by, bz))
            {
                partition[np++] = new RotationSearchSpaceRegion3DOFS8(mx, my, mz, bx, by, bz);
            }

            mxAssert(np>=0 && np<=8,"");
            return np;
        }


        //----------------------------------------------------
        //       Search State
        //----------------------------------------------------

        template<class SSR, typename Scalar=int>
        class SearchState
        {
        public:
            SSR ssr;
            Scalar bnd;
            SearchState(SSR ssr, Scalar bnd): ssr(ssr), bnd(bnd) {}
            ~SearchState(){}

            friend std::ostream &operator<<(std::ostream& os, const SearchState &ss)
            {
                os<< "ssr "<<ss.ssr<<" "<<" bnd "<<ss.bnd ;
                return os;
            }
        };


        template<class SSR, typename Scalar=int>
        class SearchStateMatches
        {
        public:
            SSR ssr;
            Scalar bnd;
            Vector3 tx;
            Vector3 ty;
            SearchStateMatches(SSR ssr, Scalar bnd, Vector3 tx, Vector3 ty):
            ssr(ssr), bnd(bnd), tx(tx), ty(ty) {}
            ~SearchStateMatches(){}

            friend std::ostream &operator<<(std::ostream& os, const SearchStateMatches &ss)
            {
                os<< "ssr "<<ss.ssr<<" "<<" bnd "<<ss.bnd ;
                return os;
            }
        };


        template<class SSR, typename Scalar=int>
        class SearchStateML
        {
        public:
            SSR ssr;
            Scalar bnd;
            int *matchList;
            int matchListSize;

            SearchStateML(SSR ssr, Scalar bnd,
                          int *matchList, int matchListSize):
            ssr(ssr), bnd(bnd), matchList(matchList),
            matchListSize(matchListSize)
            {
                mxAssert(bnd>=0 && matchListSize>=0, "invalid bound value");
            }

            friend std::ostream &operator<<(std::ostream& os, const SearchStateML &ss)
            {
                os<< "ssr "<<ss.ssr<<" "<<" bnd "<<ss.bnd <<" ml size "<<ss.matchListSize;
                return os;
            }

            ~SearchStateML()
            {
                delete []matchList;
                matchList=NULL;
            }
        };


    } // End namespace sarch
} // End namespace reg

#endif
