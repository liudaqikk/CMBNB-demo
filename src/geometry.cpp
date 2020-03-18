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

#include "geometry.h"

double reg::geometry::circleintersection(double R, double d, double r)
{    
    mxAssert(R>=0 && d>=0 && r>=0, "parametres must be positive");


    //assert(d<(R+r));
    // Return value is between 0 and pi.
    
    double rat, x, angle;
            
    if (d<=DUMMY_PRECISION)
    {
        return PI;
    }

//    if( fabs(d-(R+r))<DUMMY_PRECISION )
//    {
//        return 0;
//    }

    x = (d*d-r*r+R*R)/(2*d);

    rat = x/R;
    if (rat<=-1.0)
    {
        return PI;
    }

    angle= acos(rat);
    mxAssert(angle<=PI, "angle must be < PI");
    return angle;
}



double reg::geometry::circleIntersectionAngle(double R, double d, double r)
{
    mxAssert(R>DUMMY_PRECISION && d>DUMMY_PRECISION && r>DUMMY_PRECISION,
             "parametres must be > 0");
    // Return value is between 0 and pi.
    // Only intersection if circles touches
    // Return negative if no intersection

    const double x = (d*d-r*r+R*R)/(2.0*d);
    const double rat = x/R;
    if (rat<=-1.0)
    {
        return -1.0;
    }
    return acos(rat);
}


reg::geometry::Circle
reg::geometry::circleSterProj(const Vector3 &p, double alpha)
{
    mxAssert(( sqrt(p.x*p.x+p.y*p.y+p.z*p.z)-1.0)<DUMMY_PRECISION, "norm of p must be =1");
    mxAssert(alpha<.5*PI,"alpha <.5*pi");
    mxAssert(alpha>DUMMY_PRECISION, "alpha must be >0");

    const double psi_p = acos(p.z);
    const double psi_u = psi_p + alpha;
    const double psi_l = psi_p - alpha;

    mxAssert(psi_l<=PI && psi_u>=0, "inconsistent inclination");

    const double x= p.x;
    const double y= p.y;

    mxAssert( fabs(1-p.z) > DUMMY_PRECISION, "" );

    const double ppnorm = sqrt(x*x+ y*y);
    const double Rl_d = 1. - cos(psi_l);
    const double Ru_d = 1. - cos(psi_u);
    mxAssert(Rl_d>=0, "");
    mxAssert(Ru_d>=0, "");

    double Rl = sin(psi_l)/Rl_d;
    double Ru = sin(psi_u)/Ru_d;

    mxAssert((Rl<0 && psi_l<0) || (Rl==0 && fabs(psi_l)<DUMMY_PRECISION) || (Rl>0 && psi_l>0), "");
    mxAssert((Ru<0 && psi_u>PI) || (Rl==0 && fabs(psi_u-PI)<DUMMY_PRECISION) || (Ru>0 && psi_u<PI),"");

    Circle c;
    if (ppnorm<DUMMY_PRECISION)
    {
        c.x = 0;
        c.y = 0;
    }
    else
    {
        const double Rd = 0.5*(Rl+Ru); //centre distance from origin
        c.x = (x/ppnorm) * Rd;
        c.y = (y/ppnorm) * Rd;
    }

    c.r = fabs(.5*(Rl-Ru));
    return c;
}


reg::geometry::Circle
reg::geometry::patchSterProj(const Vector3 &p, double angle, bool &pos)
{
    mxAssert((sqrt(p.x*p.x+p.y*p.y+p.z*p.z)-1.0)<DUMMY_PRECISION, "norm of p must be 1");
    mxAssert(angle>DUMMY_PRECISION && angle < PI, "inconsistent angle");

    const double psi_p = acos(p.z);
    const double psi_u = psi_p + angle;
    const double psi_l = psi_p - angle;

    mxAssert(psi_l<=PI && psi_u>=0, "");
    pos = psi_l>=0;

    const double x = p.x;
    const double y = p.y;

    const double ppnorm = sqrt(x*x + y*y);
    const double Rl_d = 1. - cos(psi_l);
    const double Ru_d = 1. - cos(psi_u);
    mxAssert(Rl_d>=0,"");
    mxAssert(Ru_d>=0,"");

    const double Rl = sin(psi_l)/Rl_d;
    const double Ru = sin(psi_u)/Ru_d;

    mxAssert((Rl<0 && psi_l<0)  || (Rl==0 && Rl_d==0) || (Rl>0 && psi_l>0), "");
    mxAssert((Ru<0 && psi_u>PI) || (Rl==0 && Ru_d==0) || (Ru>0 && psi_u<PI), "");

    Circle c;
    if (ppnorm<DUMMY_PRECISION)
    {
        c.x = 0;
        c.y = 0;
    }
    else
    {
        const double Rd = 0.5*(Rl+Ru); //centre distance from origin. It can be negative!
        mxAssert((Rd>0&&pos) || (Rd<0&&!pos) || (Rd==0), "");

        c.x = (x/ppnorm) * Rd;
        c.y = (y/ppnorm) * Rd;
    }

    c.r = fabs(0.5*(Rl-Ru));
    return c;
}


reg::geometry::HalfPlane
reg::geometry::halfPlaneSterProj(const Vector3 &p, double alpha)
{
    mxAssert(alpha>DUMMY_PRECISION && alpha<PI/2., "inconsistent angle");

    const double inv_d = tan(alpha); // 1/distance
    const double norm_xy = sqrt(p.x*p.x + p.y*p.y); // xy-norm

    mxAssert(inv_d>DUMMY_PRECISION && norm_xy>DUMMY_PRECISION && inv_d*norm_xy>DUMMY_PRECISION, "");
    HalfPlane hp( p.x/(norm_xy*inv_d),
                  p.y/(norm_xy*inv_d));

    return hp;
}

