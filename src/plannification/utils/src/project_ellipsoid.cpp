/**
 These two C-routines calculate the projection P of a point W on the ellipse,
 or on the surface of an Ellipsoid in 3D.
 The ellipse/ellipsoid is aligned with the principal axes (X, Y, Z),
 and has radii (radX, radY, radZ).

 Francois Nedelec. Copyright 2007-2017 EMBL.
 
 This code is Open Source covered by the GNU GPL v3.0 License
*/

#include "utils/project_ellipsoid.h"

#include <stdlib.h>
#include <vector>

/**
 Calculate the projection P = (pX, pY) of the point W = (wX, wY) on the ellipse that
 is aligned with the X and Y axis, and has radii (radX, radY).
 
 Method:
 
 A vector orthogonal to the ellipse at position (X, Y) is
 
     N = ( X / radX^2, Y / radY^2 ),
 
 and we can thus write W = P + h * N, leading to:

     pX = wX * radX^2 / ( radX^2 + h );
     pY = wY * radY^2 / ( radY^2 + h );

 if wX and wY are not both null.
 
 Moreover, the projection should be on the ellipse and thus `h` should be a zero of:
 
     F(h) = ( pX / radX )^2 + ( pY / radY )^2 - 1
 
 We follow Newton's rule to find the root of F(h), and use the formula above to
 calculate the projection.
 */
void projectEllipse(double& pX,  double& pY,
                    double wX,   double wY,
                    double radX, double radY)
{
    // handle the pathological cases:
    if ( wX == 0 )
    {
        pX = 0;
        pY = copysign(radY, wY);
        return;
    }
    if ( wY == 0 )
    {
        pX = copysign(radX, wX);
        pY = 0;
        return;
    }
    
    double aa = radX * radX;
    double bb = radY * radY;
    
    // we derive a lower limit for 'h' from  pX^2 + pY^2 > max(radX,radY)^2
    double RR = fmax(aa, bb);
    // 'hmin' is the minimum value that 'h' could have
    double hmin = sqrt( ( wX*wX*aa*aa + wY*wY*bb*bb ) / RR ) - RR;
    
    // we derive another lower limit for 'h' from  |pX| < radX
    hmin = fmax(hmin, ( fabs(wX) - radX ) * radX);

    // we derive another lower limit for 'h' from  |pY| < radY
    hmin = fmax(hmin, ( fabs(wY) - radY ) * radY);

    // if the point is outside, then 'h' should be positive:
    if ( wX*wX/aa + wY*wY/bb > 1  &&  hmin < 0 )
        hmin = 0;
    
    double h_old, h = hmin;

    //fprintf(stderr, " <<< %+.10f  %+.10f    hmin %+10.4f", wX, wY, hmin);

    // follow Newton's iteration to find the root
    unsigned cnt = 0;
    do {
        double aah = aa + h;
        double bbh = bb + h;

        double waX = wX / aah;
        double waY = wY / bbh;
        
        double pXX = waX * waX * aa;
        double pYY = waY * waY * bb;

        h_old = h;
        
        double F    = 1 - ( pXX         + pYY       );
        double dF   = 2 * ( pXX / aah   + pYY / bbh );

        // Newtons' method
        h -= F / dF;
        
        //fprintf(stderr, "  %i : h %+f  F %+20.16f  dF %+20.16f  dh %e\n", cnt, h, F, dF, h-h_old);
        
        if ( h < hmin )
        {
            h = 0.5 * ( h_old + hmin );
            continue;
        }

        if ( ++cnt > 20 )
            break;
        
    } while ( h > h_old );

    // calculate the projection from h
    pX = wX * aa / ( aa + h );
    pY = wY * bb / ( bb + h );
}


/**
 Calculates the projection P = (pX, pY, pZ) of the point W = (wX, wY, wZ) on the ellipse that
 is aligned with the X and Y axis, and has radii (radX, radY, radZ).
 
 Method:
 
 A vector orthogonal to the ellipse at position ( X, Y, Z ) is
 
    N = ( X / radX^2, Y / radY^2, Z / radZ^2 ),
 
 and we can thus write W = P + h * N, for some scalar `h` leading to:

    pX = wX / ( 1 + h / radX^2 );
    pY = wY / ( 1 + h / radY^2 );
    pZ = wZ / ( 1 + h / radZ^2 );
 
 Moreover, the projection should be on the ellipse and thus `h` should be a zero of:

     F(h) = ( pX / radX )^2 + ( pY / radY )^2 + ( pZ / radZ )^2 - 1
 
 We follow Newton's rule to find the root of F(h), and use the formula above to
 calculate the projection.
 */
void projectEllipsoid(std::array<double,3>& p,
                      const std::array<double,3> w,
                      const std::array<double,3> rad)
{
    assert( rad[0]==rad[0] && rad[0] >= 0 );
    assert( rad[1]==rad[1] && rad[1] >= 0 );
    assert( rad[2]==rad[2] && rad[2] >= 0 );

    if(rad[0] == 0)
    {
        p[0] = w[0];
        p[1] = w[1];
        p[2] = w[2];
        return;
    }
    
    // handle the pathological cases:
    if ( w[0] == 0 )
    {
        p[0] = 0;
        projectEllipse(p[1], p[2], w[1], w[2], rad[1], rad[2]);
        return;
    }
    if ( w[1] == 0 )
    {
        p[1] = 0;
        projectEllipse(p[0], p[2], w[0], w[2], rad[0], rad[2]);
        return;
    }
    if ( w[2] == 0 )
    {
        p[2] = 0;
        projectEllipse(p[0], p[1], w[0], w[1], rad[0], rad[1]);
        return;
    }

    double aa = rad[0] * rad[0];
    double bb = rad[1] * rad[1];
    double cc = rad[2] * rad[2];

    // we derive a lower limit for 'h' from  pX^2 + pY^2 + pZ^2 < max(radX,radY,radZ)^2
    double RR = fmax(aa, fmax(bb,cc));
    // 'hmin' is the minimum value that 'h' can have
    double hmin = sqrt( ( w[0]*w[0]*aa*aa + w[1]*w[1]*bb*bb + w[2]*w[2]*cc*cc ) / RR ) - RR;

    // we derive another lower limit for 'h' from  |pX| < radX
    hmin = fmax(hmin, ( fabs(w[0]) - rad[0] ) * rad[0]);

    // we derive another lower limit for 'h' from  |pY| < radY
    hmin = fmax(hmin, ( fabs(w[1]) - rad[1] ) * rad[1]);
    
    // we derive another lower limit for 'h' from  |pZ| < radZ
    hmin = fmax(hmin, ( fabs(w[2]) - rad[2] ) * rad[2]);

    if ( w[0]*w[0]/aa + w[1]*w[1]/bb + w[2]*w[2]/cc > 1  &&  hmin < 0 )
    {
        // if the point is outside, then 'h' should be positive:
        hmin = 0;
    }

    double h_old, h = hmin;
    //fprintf(stderr, "----- h %+f\n", h);

    /*
     Follow Newton's iteration to find the largest root.
     We start with h>0, and h should only increase
     */
    unsigned cnt = 0;
    do {
        double aah = aa + h;
        double bbh = bb + h;
        double cch = cc + h;

        double waX = w[0] / aah;
        double waY = w[1] / bbh;
        double waZ = w[2] / cch;
        
        double pXX = waX * waX * aa;
        double pYY = waY * waY * bb;
        double pZZ = waZ * waZ * cc;

        h_old = h;

        double   F = 1 - ( pXX         + pYY         + pZZ       );
        double  dF = 2 * ( pXX / aah   + pYY / bbh   + pZZ / cch );

        // Newton's method
        h -= F / dF;
        
        //fprintf(stderr, "  %i : h %+f  F %+e dh %+.20f\n", cnt, h_old, F, h-h_old);
        //fprintf(stderr, "       %+.10f   %+.10f   %+.10f   %+.10f\n", F, F/dF, ddF/dF, dddF/dF);

        if ( h < hmin )
        {
            h = 0.5 * ( h_old + hmin );
            continue;
        }

        if ( ++cnt > 20 )
            break;
        
    } while ( h > h_old );

    // calculate the projection from h
    p[0] = w[0] * aa / ( aa + h );
    p[1] = w[1] * bb / ( bb + h );
    p[2] = w[2] * cc / ( cc + h );
}

void getSurfacePoint(std::vector<double>& pt,
                      const std::vector<double> &angles,
                      const std::vector<double> &center,
                      const std::vector<double> &rad)
{
    /**
     * Ellipsoid parametric representation
     * x=xc​+rx ​sin(ϕ)cos(θ)
     * y=yc+ry sin⁡(ϕ)sin⁡(θ)
     * z=zc+rz cos⁡(ϕ)
     * θ : longitude
     * ϕ : latitude
     */

    if(rad.size() == 2) // 2D
    {
        pt.push_back(center[0] + rad[0]*sin(angles[1])*cos(angles[0])); // x
        pt.push_back(center[1] + rad[1]*sin(angles[1])*sin(angles[0])); // y
        pt.push_back(0.0); // z
    }
    else // 3D
    {
        pt.push_back(center[0] + rad[0]*sin(angles[1])*cos(angles[0])); // x
        pt.push_back(center[1] + rad[1]*sin(angles[1])*sin(angles[0])); // y
        pt.push_back(center[2] + rad[2]*cos(angles[1])); // z
    }
}