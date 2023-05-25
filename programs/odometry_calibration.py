
import math
import numpy as np
import scipy
import matplotlib.pyplot as plt


def carmen_global_convert_degmin_to_double(dm_format: float) -> float:
    degree = math.floor(dm_format / 100.0)
    minutes = (dm_format - degree * 100.0) / 60.0
    return degree + minutes


class Utm_Coord_3d:
    def __init__(self, X: float = 0.0, Y: float = 0.0, Z: float = 0.0, Zone: int = 0, hemi_n: bool = False):
        self.x = X
        self.y = Y
        self.z = Z
        self.zone = Zone
        self.hemisphere_north = hemi_n


class Gdc_Coord_3d:
    def __init__(self, lat: float, lon: float, e: float) -> None:
        self.latitude = lat
        self.longitude = lon
        self.elevation = e


class Gdc_To_Utm_Converter:
    RADIANS_PER_DEGREE = 0.0174532925199432957692
    PI = 4.0 * math.atan(1.0)
    CScale = .9996
    init = False

    @staticmethod
    def CreateConstants(a: float, f: float):
        Gdc_To_Utm_Converter.A = a
        Gdc_To_Utm_Converter.F = f

        # Create the ERM constants.
        Gdc_To_Utm_Converter.F = 1 / (Gdc_To_Utm_Converter.F)
        Gdc_To_Utm_Converter.C = Gdc_To_Utm_Converter.A * \
            (1 - Gdc_To_Utm_Converter.F)
        Gdc_To_Utm_Converter.Eps2 = Gdc_To_Utm_Converter.F * \
            (2.0 - Gdc_To_Utm_Converter.F)
        Gdc_To_Utm_Converter.Eps25 = .25 * (Gdc_To_Utm_Converter.Eps2)
        Gdc_To_Utm_Converter.Epps2 = (
            Gdc_To_Utm_Converter.Eps2) / (1.0 - Gdc_To_Utm_Converter.Eps2)
        Gdc_To_Utm_Converter.polx2b = 1.0 * Gdc_To_Utm_Converter.Eps2 + \
            1.0 / 4.0 * math.pow(Gdc_To_Utm_Converter.Eps2, 2) + 15.0/128.0 * \
            math.pow(Gdc_To_Utm_Converter.Eps2, 3) - \
            455.0/4096.0 * math.pow(Gdc_To_Utm_Converter.Eps2, 4)

        Gdc_To_Utm_Converter.polx2b = 3.0/8.0 * Gdc_To_Utm_Converter.polx2b

        Gdc_To_Utm_Converter.polx3b = 1.0 * pow(Gdc_To_Utm_Converter.Eps2, 2) + 3.0/4.0 * \
            math.pow(Gdc_To_Utm_Converter.Eps2, 3) - 77.0 / \
            128.0 * math.pow(Gdc_To_Utm_Converter.Eps2, 4)

        Gdc_To_Utm_Converter.polx3b = 15.0 / 256.0 * Gdc_To_Utm_Converter.polx3b

        Gdc_To_Utm_Converter.polx4b = pow(Gdc_To_Utm_Converter.Eps2, 3) - 41.0 / \
            32.0 * pow(Gdc_To_Utm_Converter.Eps2, 4)

        Gdc_To_Utm_Converter.polx4b = Gdc_To_Utm_Converter.polx4b * 35.0 / 3072.0

        Gdc_To_Utm_Converter.polx5b = -315.0 / \
            131072.0 * math.pow(Gdc_To_Utm_Converter.Eps2, 4)

        Gdc_To_Utm_Converter.poly1b = 1.0 - \
            (1.0/4.0 * Gdc_To_Utm_Converter.Eps2) - (3.0/64.0 * math.pow(Gdc_To_Utm_Converter.Eps2, 2)) - \
            (5.0/256.0 * math.pow(Gdc_To_Utm_Converter.Eps2, 3)) - \
            (175.0/16384.0 * math.pow(Gdc_To_Utm_Converter.Eps2, 4))

        Gdc_To_Utm_Converter.poly2b = Gdc_To_Utm_Converter.polx2b * -2.0 + Gdc_To_Utm_Converter.polx3b * 4.0 - Gdc_To_Utm_Converter.polx4b * \
            6.0 + Gdc_To_Utm_Converter.polx5b * 8.0

        Gdc_To_Utm_Converter.poly3b = Gdc_To_Utm_Converter.polx3b * -8.0 + Gdc_To_Utm_Converter.polx4b * \
            32.0 - Gdc_To_Utm_Converter.polx5b * 80.0

        Gdc_To_Utm_Converter.poly4b = Gdc_To_Utm_Converter.polx4b * - \
            32.0 + Gdc_To_Utm_Converter.polx5b * 192.0

        Gdc_To_Utm_Converter.poly5b = Gdc_To_Utm_Converter.polx5b * -128.0

    @staticmethod
    def Convert(gdc: Gdc_Coord_3d) -> Utm_Coord_3d:
        if not Gdc_To_Utm_Converter.init:
            Gdc_To_Utm_Converter.CreateConstants(6378137, 298.257223563)
            Gdc_To_Utm_Converter.init = True

        utm = Utm_Coord_3d()
        utm.z = gdc.elevation

        if (gdc.latitude < 0):
            utm.hemisphere_north = False
        else:
            utm.hemisphere_north = True

        # if (gdc.longitude < 0.0) // XXX - reddy, 11 Sep 98
        # gdc.longitude += 360.0

        source_lat = gdc.latitude * Gdc_To_Utm_Converter.RADIANS_PER_DEGREE
        source_lon = gdc.longitude * Gdc_To_Utm_Converter.RADIANS_PER_DEGREE

        s1 = math.sin(source_lat)
        c1 = math.cos(source_lat)
        tx = s1 / c1
        s12 = s1 * s1

        # USE IN-LINE SQUARE ROOT
        rn = Gdc_To_Utm_Converter.A / ((.25 - Gdc_To_Utm_Converter.Eps25*s12 + .9999944354799/4) +
                                       (.25-Gdc_To_Utm_Converter.Eps25*s12)/(.25 - Gdc_To_Utm_Converter.Eps25*s12 + .9999944354799/4))

        # COMPUTE UTM COORDINATES

        # Compute Zone
        utm.zone = int(source_lon * 30.0 / Gdc_To_Utm_Converter.PI + 31)

        if (utm.zone <= 0):
            utm.zone = 1
        else:
            if (utm.zone >= 61):
                utm.zone = 60

        axlon0 = (utm.zone * 6 - 183) * Gdc_To_Utm_Converter.RADIANS_PER_DEGREE

        al = (source_lon - axlon0) * c1

        sm = s1 * c1 * (Gdc_To_Utm_Converter.poly2b + s12 * (Gdc_To_Utm_Converter.poly3b + s12 *
                                                             (Gdc_To_Utm_Converter.poly4b + s12 * Gdc_To_Utm_Converter.poly5b)))

        sm = Gdc_To_Utm_Converter.A * \
            (Gdc_To_Utm_Converter.poly1b * source_lat + sm)

        tn2 = tx * tx
        cee = Gdc_To_Utm_Converter.Epps2 * c1 * c1
        al2 = al * al
        poly1 = 1.0 - tn2 + cee
        poly2 = 5.0 + tn2 * (tn2 - 18.0) + cee * (14.0 - tn2 * 58.0)

        # COMPUTE EASTING
        utm.x = Gdc_To_Utm_Converter.CScale * rn * al * \
            (1.0 + al2 * (.166666666666667 * poly1 + .00833333333333333 * al2 * poly2))

        utm.x += 5.0E5

        # COMPUTE NORTHING

        poly1 = 5.0 - tn2 + cee * (cee * 4.0 + 9.0)
        poly2 = 61.0 + tn2 * (tn2 - 58.0) + cee * (270.0 - tn2 * 330.0)

        utm.y = Gdc_To_Utm_Converter.CScale * (sm + rn * tx * al2 * (0.5 + al2 *
                                                                     (.0416666666666667 * poly1 + .00138888888888888 * al2 * poly2)))

        if (source_lat < 0.0):
            utm.y += 1.0E7

        return utm


def parse_gps_position(line: str):
    line = line.strip().split()
    lt = carmen_global_convert_degmin_to_double(float(line[3]))
    lg = carmen_global_convert_degmin_to_double(float(line[5]))

    # verify the latitude and longitude orientations
    if ('S' == line[4][0]):
        lt = -lt
    if ('W' == line[6][0]):
        lg = -lg

    # convert to x and y coordinates
    gdc = Gdc_Coord_3d(lt, lg, float(line[10]))

    # Transformando o z utilizando como altitude a altitude mesmo - que esta vindo como zero
    utm = Gdc_To_Utm_Converter.Convert(gdc)

    offset_x = 7757733.604602944
    offset_y = -363559.0912591674

    gps_x = utm.y - offset_x
    gps_y = -utm.x - offset_y

    gps_quality = int(line[7])
    gps_time = float(line[len(line) - 3])

    return gps_x, gps_y, gps_time, gps_quality


def parse_odometry(line):
    line = line.split()
    v = float(line[1])
    phi = float(line[2])
    t = float(line[3])
    return v, phi, t


class DeadReckoning:
    dist_front_rear_axles = 2.625

    def __init__(self, x=0, y=0, theta=0):
        self._initial_pose = [x, y, theta]
        self.reset()

    def reset(self):
        self._prev_time = -1
        self.pose = np.copy(self._initial_pose)

    def update(self, v: float, phi: float, t: float):
        if self._prev_time == -1:
            self._prev_time = t
            return self.pose

        # ackermann model
        dt_v = (t - self._prev_time) * v
        self.pose[0] += dt_v * math.cos(self.pose[2])
        self.pose[1] += dt_v * math.sin(self.pose[2])
        self.pose[2] += dt_v * \
            math.tan(phi) / DeadReckoning.dist_front_rear_axles

        self._prev_time = t

        return self.pose


def optimize_params(gpss, odom, init_angle):
    nearest_poses = []
    for (x, y, t, _) in gpss:
        # Since the sensors work in different frequencies, for each dead reckoning pose, we find the closest gps pose
        ts = [odom_t - t for _, _, odom_t in odom]
        nearest_poses.append(np.argmin(ts))

    def error_fn(params):
        v_mult, phi_mult, phi_offset = params
        dead_reckoning = DeadReckoning(theta=init_angle)

        dr_poses = np.array([
            np.copy(
                dead_reckoning.update(
                    v * v_mult,
                    phi * phi_mult + phi_offset,
                    t
                ))
            for v, phi, t in odom
        ])

        dr_times = np.array([t for (_, _, t) in odom])
        dr_poses = np.hstack((dr_poses, dr_times[:, None]))

        # sum of euclidean distances.
        error = 0

        for (x, y, t, _), idx in zip(gpss, nearest_poses):
            # Since the sensors work in different frequencies, for each dead reckoning pose, we find the closest gps pose
            closest_pose = dr_poses[idx]
            error += np.sqrt((x - closest_pose[0])
                             ** 2 + (y - closest_pose[1])**2)

        return error

    return scipy.optimize.differential_evolution(error_fn, bounds=[[0.7, 1.3], [0.7, 1.3], [-0.3, 0.3]], disp=True, workers=-1)


def find_gps_angle(gpss):
    for i in range(len(gpss)):
        d = math.sqrt((gpss[i, 0] - gpss[0, 0]) ** 2 +
                      (gpss[i, 1] - gpss[0, 1]) ** 2)
        if d > 2:
            return math.atan2(gpss[i, 1] - gpss[0, 1], gpss[i, 0] - gpss[0, 0])
    return 0


def main():
    data = open(
        "/home/filipe/data/logs_iara/log-volta-da-ufes-20181206.txt", "r").readlines()

    gpss = []
    odom = []

    for line in data:
        if line.startswith("NMEAGGA 1"):  # 1 is the gps id
            gpss.append(parse_gps_position(line))

        if line.startswith("ROBOTVELOCITY_ACK"):
            odom.append(parse_odometry(line))

    gpss = np.array(gpss)
    odom = np.array(odom)
    init_angle = find_gps_angle(gpss)
    print(init_angle)

    dead_reckoning = DeadReckoning(theta=init_angle)
    dr_poses = np.array([np.copy(dead_reckoning.update(v, phi, t))
                         for v, phi, t in odom])

    # result = optimize_params(gpss, odom, init_angle)
    # print("Resulst:", result)
    # calib = result["x"]
    calib = [1.0, 0.975, -0.004]

    dead_reckoning.reset()
    calib_dr_poses = np.array([np.copy(dead_reckoning.update(v * calib[0], phi * calib[1] + calib[2], t))
                               for v, phi, t in odom])

    plt.plot(gpss[:, 0], gpss[:, 1], '.')
    # plt.plot(dr_poses[:, 0], dr_poses[:, 1], '.')
    plt.plot(calib_dr_poses[:, 0], calib_dr_poses[:, 1], '.')

    print("Done.")
    plt.show()


if __name__ == "__main__":
    main()
