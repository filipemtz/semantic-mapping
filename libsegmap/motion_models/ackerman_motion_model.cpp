
#include <cstdio>
#include <cstdlib>
#include "libsegmap/libcarmen_util/util_math.h"
#include "libsegmap/motion_models/ackerman_motion_model.h"
#include "libsegmap/types/segmap_conversions.h"
#include "libsegmap/types/segmap_definitions.h"


void
ackerman_motion_model(double &x, double &y, double &th, double v, double phi, double dt)
{
	if (fabs(phi) > degrees_to_radians(80))
		exit(printf("Error phi = %lf\n", radians_to_degrees(phi)));

	double ds = dt * v;

	th += (ds / distance_between_front_and_rear_axles) * tan(phi);
	th = normalize_theta(th);
	x += ds * cos(th);
	y += ds * sin(th);
}


void
ackerman_motion_model(Pose2d &pose, double v, double phi, double dt)
{
	ackerman_motion_model(pose.x, pose.y, pose.th, v, phi, dt);
}
