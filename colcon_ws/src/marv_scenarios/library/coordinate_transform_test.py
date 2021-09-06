import numpy as np

# Transforms geodetic coordinates in the WGS84 (World Geodetic System 1984) coordinate frame 
# to local navigation coordinates in the ENU (East North Up) cordinate frame
# Algorithm taken from https://www.researchgate.net/publication/27253833_Converting_GPS_coordinates_phi_lambda_h_to_navigation_coordinates_ENU
# llh0: reference point in geodetic coordinates np.array(((lat),(long),(alt)))
# llh:  data point in geodetic coordinates -||-
def WGS84_to_ENU(llh0, llh):
    # Constants
    a = 6378137 # Length of earth's semi-major axis
    b = 6356752.3142 # Length of earth's semi-minor axis
    e2 = 1 - np.power(b/a,2)

    # Location of reference point in radians
    phi = np.deg2rad(llh0[0])
    lam = np.deg2rad(llh0[1])
    h = llh0[2]

    # Location of data points in radians
    dphi = np.deg2rad(llh[0]) - phi
    dlam = np.deg2rad(llh[1]) - lam
    dh = llh[2] - h

    # Definitions
    tmp1 = np.sqrt(1-e2*np.power(np.sin(phi),2))
    cl = np.cos(lam)
    sl = np.sin(lam)
    cp = np.cos(phi)
    sp = np.sin(phi)

    # Transformations
    de = (a/tmp1+h)*cp*dlam - (a*(1-e2)/(np.power(tmp1,3)) + h)*sp*dphi*dlam + cp*dlam*dh

    dn = (a*(1-e2)/np.power(tmp1,3) + h)*dphi + 1.5*cp*sp*a*e2*np.power(dphi,2) + np.power(sp,2)*dh*dphi + 0.5*sp*cp*(a/tmp1 + h)*np.power(dlam,2)

    du = dh - 0.5*(a-1.5*a*e2*np.power(cp,2) + 0.5*a*e2 + h)*np.power(dphi,2) - 0.5*np.power(cp,2)*(a/tmp1 - h)*np.power(dlam,2)

    return np.array(((de),(dn),(du)))

# Transforms a ENU (East North Up) to a NED (North East Down) coordinate frame
def ENU_to_NED(ENU_coords):
    NED_coords = np.array(((ENU_coords[1]),(ENU_coords[0]),(-ENU_coords[2])))
    
    return NED_coords


reference_position_geo = np.array([57.6667, 11.8333, 0.0])
current_pos_geo = np.array([57.662215, 11.833556, 0])

current_pos_nav = ENU_to_NED(WGS84_to_ENU(reference_position_geo, current_pos_geo))
print(current_pos_nav)