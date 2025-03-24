/**
 These two C-routines calculate the projection P of a point W on the ellipse,
 or on the surface of an Ellipsoid in 3D.
 The ellipse/ellipsoid is aligned with the principal axes (X, Y, Z),
 and has radii (radX, radY, radZ).

 Francois Nedelec. Copyright 2007-2017 EMBL.
 
 This code is Open Source covered by the GNU GPL v3.0 License
 */
 
#pragma once

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <array>
#include <vector>
#include <iostream>

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
                    double radX, double radY);


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
                      const std::array<double,3> rad);

/** \brief Get the point on the surface ( @e pt) of an ellipsoid of semi axis @e rad, center at @e center according to the ellipsoid parameteric representation 
 * using the two angles @e angles
*/
void getSurfacePoint(std::vector<double>& pt,
                      const std::vector<double> &angles,
                      const std::vector<double> &center,
                      const std::vector<double> &rad);

/** \brief Function to check if the point is inside the ellipsoid and project if necessary */
std::vector<double> checkAndProject(const std::vector<double>& point, const std::vector<double>& center, const std::vector<double>& semiAxes) 
{
    // Compute the scaled distance
    double distance = 0.0;
    for (size_t i = 0; i < semiAxes.size(); ++i) 
    {
        double diff = (point[i] - center[i]) / semiAxes[i];
        distance += diff * diff;
    }

    // Check if the point is inside or on the ellipsoid
    if (distance <= 1.0) 
    {
        // Point is inside the ellipsoid
        return point;
    } 
    else 
    {
        // Point is outside; compute the projection onto the ellipsoid surface
        double lambda_factor = 1.0 / std::sqrt(distance);
        std::vector<double> projectedPoint(semiAxes.size());
        for (size_t i = 0; i < semiAxes.size(); ++i) {
            projectedPoint[i] = center[i] + lambda_factor * (point[i] - center[i]);
        }

        double project_dist = 0.0;
        for (size_t i = 0; i < semiAxes.size(); ++i) 
        {
            double diff = (projectedPoint[i] - center[i]) / semiAxes[i];
            project_dist += diff * diff;
        }

        assert(project_dist <= 1.1 && "Ellipsoid projection failed ! Projected distance = " && project_dist);

        return projectedPoint;
    }
}
