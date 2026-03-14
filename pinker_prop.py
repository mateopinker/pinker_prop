import math

AIRFOIL_POLARS = {
    "s8025": (
        (50000, 5.75, 0.6472, 0.02431),
        (100000, 6.25, 0.7023, 0.01889),
        (200000, 6.50, 0.7340, 0.01501),
        (500000, 7.00, 0.7928, 0.01217),
        (1000000, 6.75, 0.7713, 0.00989),
    ),

    "a18": (
        (40400, 6.98, 1.002, 0.0346),
        (61300, 7.55, 1.049, 0.0268),
        (101700, 7.54, 1.014, 0.0264),
        (203800, 7.44, 1.033, 0.0215),
        (303300, 9.25, 1.138, 0.0260),
    ),

    "naca0012": (
        (10000, 6.5625, 0.4, 0.08),
        (20000, 8.75, 0.5, 0.13),
        (30000, 7.5, 0.6, 0.08),
        (50000, 8.125, 0.675, 0.05),
        (60000, 7.5, 0.7, 0.04),
        (100000, 7.5, 0.72, 0.037)
    ),
    # Add more airfoils here with the same point format:
    # "airfoil_name": ((Re, alpha_opt_deg, Cl_opt, Cd_opt), ...),
}


def normalize_airfoil_name(airfoil):
    """
    Normalizes the airfoil selector and validates it against the loaded data.
    """
    airfoil_key = airfoil.strip().lower()

    if airfoil_key not in AIRFOIL_POLARS:
        raise ValueError(
            "airfoil must be one of: " + ", ".join(list_available_airfoils())
        )

    return airfoil_key


def list_available_airfoils():
    """
    Returns the airfoils that have polar data available in this script.
    """
    return tuple(sorted(AIRFOIL_POLARS.keys()))


def inflow(outer_d, total_mass_kg, rho=1.225, g=9.81):
    """
    Computes inflow V_p for a quadrotor in hover
    using actuator disk (momentum) theory.
    """
    # Total weight
    W = total_mass_kg * g  # [N]

    # Thrust per propeller (quadrotor)
    F_prop = W / 4  # [N]

    # Disk area
    A = math.pi * (outer_d / 2)**2  # [m^2]

    # Inflow velocity from momentum theory
    Vp = math.sqrt(F_prop / (2 * rho * A))

    return Vp , F_prop

def thrust_distribution(outer_d, hub_d, F_prop, n_blades, sections, distribution="uniform"):
    """
    Thrust distribution discretized into 'sections' radial elements.

    Supported distributions:
        "uniform" : constant dT/dr along the span
        "ramp"    : linearly increasing dT/dr from shaft axis minimum to tip maximum

    Numeric selectors are also accepted:
        0 -> "uniform"
        1 -> "ramp"

    Returns:
        F_blade : thrust per blade [N]
        dT_list : list of thrust per radial section [N]
    """
    outer_r = outer_d / 2
    root_r  = hub_d / 2
    span = outer_r - root_r
    dr = span / sections

    F_blade = F_prop / n_blades # [N]

    if distribution == 0:
        distribution = "uniform"
    elif distribution == 1:
        distribution = "ramp"

    distribution = distribution.lower()

    dT_list = []
    n = 0

    if distribution == "uniform":
        dT_dr = F_blade / span # [N/m]

        while n != sections:
            dT = dT_dr * dr # thrust in this section [N]
            dT_list.append(dT)
            n = n + 1

    elif distribution == "ramp":
        # dT/dr grows linearly with absolute radius, referenced to the shaft axis.
        # This makes the theoretical minimum occur at r = 0, not at the blade root.
        slope = 2 * F_blade / (outer_r**2 - root_r**2) # [N/m^2]

        while n != sections:
            r_inner = root_r + dr * n
            r_outer = root_r + dr * (n + 1)
            dT = 0.5 * slope * (r_outer**2 - r_inner**2)
            dT_list.append(dT)
            n = n + 1

    else:
        raise ValueError("distribution must be 'uniform', 'ramp', 0, or 1")

    return F_blade, dT_list

def relative_velocity(motor_rpm,outer_d,hub_d,sections,Vp):
    """
    Outputs a list of relative velocities at each radial station
    given uniform inflow velocity distribution and discretization of the blade.
    """
    n = 0
    v_relative_module = [] #list of relative velocity magnitude
    v_relative_angle = [] #list of relative velocity angle
    outer_r = outer_d / 2
    root_r = hub_d/2
    span = outer_r - root_r
    dr = span / sections

    while n != sections:
        radius = root_r + dr * (n + 0.5) #radial station [m]
        V_tan = 2*math.pi* motor_rpm / 60* radius #tangential velocity [m/s]
        V_magnitude = math.sqrt(Vp**2 + V_tan**2) #relative velocity magnitude [m/s]
        V_angle = math.atan2(Vp, V_tan) #relative velocity angle [rad] /tan2 because it works even for V_tan = 0

        v_relative_module.append(V_magnitude)
        v_relative_angle.append(V_angle)
        n = n+1

    return v_relative_module, v_relative_angle

def interp_airfoil_polar(Re, airfoil="s8025"):
    """
    Linear interpolation of the polars for the selected airfoil.
    For a given reynolds returns optimal angle of attack (deg),
    lift coefficient, and drag coefficient. (alpha_opt_deg, Cl_opt, Cd_opt
    NO EXTRAPOLATION BEYOND DATA RANGE.
    """
    airfoil_key = normalize_airfoil_name(airfoil)
    polar_points = AIRFOIL_POLARS[airfoil_key]

    Re0, a0, cl0, cd0 = polar_points[0]
    ReN, aN, clN, cdN = polar_points[-1]

    # I clamp data range (no extrapolation)
    if Re <= Re0:
        return a0, cl0, cd0
    if Re >= ReN:
        return aN, clN, cdN

    n = 0

    while n != len(polar_points) - 1:
        Re_low, a_low, cl_low, cd_low = polar_points[n]
        Re_high, a_high, cl_high, cd_high = polar_points[n + 1]

        if Re <= Re_high:
            t = (Re - Re_low) / (Re_high - Re_low)
            return (
                a_low + t * (a_high - a_low),
                cl_low + t * (cl_high - cl_low),
                cd_low + t * (cd_high - cd_low),
            )

        n = n + 1

    return aN, clN, cdN


def interp_s8025_polar(Re):
    """
    Compatibility wrapper for existing code paths that still call S8025 directly.
    """
    return interp_airfoil_polar(Re, "s8025")


def section_forces(chord, V_rel, V_angle, dr, rho=1.225,nu=1.46e-5, airfoil="s8025"):
        """
        Local aerodynamic solution for one blade element at a given chord.
        """
        Re = chord * V_rel/ nu
        AoA, Cl, Cd = interp_airfoil_polar(Re, airfoil)
        dT = 0.5 * rho * V_rel**2 * chord * Cl * dr*math.cos(V_angle) - 0.5 * rho * V_rel**2 * chord * Cd * dr*math.sin(V_angle)
        dD = 0.5 * rho * V_rel**2 * chord * Cd * dr

        return AoA, Cl, Cd, Re, dT, dD


def redistribute_missing_thrust(top_bracket,dT_list, v_relative_module,v_relative_angle,outer_d, hub_d, sections,rho=1.225,nu=1.46e-5, airfoil="s8025"):
        """
        If an element needs more than the available thrust at max chord,
        I cap that element at top_bracket and redistribute the missing thrust
        among the remaining free elements, preserving their current relative loading.
        """
        redistributed_dT = []
        capped = []

        n = 0
        while n != sections:
            redistributed_dT.append(dT_list[n])
            capped.append(False)
            n = n + 1

        outer_r = outer_d / 2
        root_r  = hub_d / 2
        span = outer_r - root_r
        dr = span / sections

        while True:
            missing_thrust = 0
            n = 0

            while n != sections:
                if capped[n] == False:
                    V_rel = v_relative_module[n]
                    V_angle = v_relative_angle[n]
                    AoA_high, Cl_high, Cd_high, Re_high, dT_high, dD_high = section_forces(top_bracket, V_rel, V_angle, dr, rho, nu, airfoil)

                    if redistributed_dT[n] > dT_high:
                        print("Element", n + 1, "hit max chord:", top_bracket, "m. Missing thrust:", redistributed_dT[n] - dT_high, "N")
                        missing_thrust = missing_thrust + redistributed_dT[n] - dT_high
                        redistributed_dT[n] = dT_high
                        capped[n] = True

                n = n + 1

            if missing_thrust == 0:
                break

            free_thrust = 0
            n = 0

            while n != sections:
                if capped[n] == False:
                    free_thrust = free_thrust + redistributed_dT[n]
                n = n + 1

            if free_thrust == 0:
                raise ValueError("max chord is too small to meet the required thrust")

            n = 0
            while n != sections:
                if capped[n] == False:
                    redistributed_dT[n] = redistributed_dT[n] + missing_thrust * redistributed_dT[n] / free_thrust
                n = n + 1

        return redistributed_dT, capped


def polarpicker(top_bracket, bottom_bracket,dT_list, v_relative_module,v_relative_angle,exit_step,outer_d, hub_d, sections,rho=1.225,nu=1.46e-5, airfoil="s8025"):
        """
        Chord and AoA distribution calculation.
        I compute dT=dLcos(phi) - dDsin(phi) and
        dL =  0.5 * rho *V_rel**2 * chord * Cl *dr
        dD = 0.5 * rho *V_rel**2 * chord * Cd * dr
        I use the bisection method to find the chord that satisfies the equation for each element
        If an element exceeds the max chord, I cap it and redistribute its missing thrust.
        """
        
        thrust = []
        AoA = []
        chords = [] #[m]
        drag = []
        Reynolds = []

        n=0
        mid_bracket = (top_bracket + bottom_bracket)/2
        bottom_og = bottom_bracket
        top_og = top_bracket

        outer_r = outer_d / 2
        root_r  = hub_d / 2
        span = outer_r - root_r
        dr = span / sections

        redistributed_dT, capped = redistribute_missing_thrust(top_bracket,dT_list, v_relative_module,v_relative_angle,outer_d, hub_d, sections,rho,nu, airfoil)



        while n != sections:
            dT = redistributed_dT[n]
            V_rel = v_relative_module[n]
            V_angle = v_relative_angle[n]

            if capped[n] == True:
                a_high, Cl_high, Cd_high, Re_high, dT_high, dD_high = section_forces(top_bracket, V_rel, V_angle, dr, rho, nu, airfoil)
                thrust.append(dT_high)
                AoA.append(a_high)
                chords.append(top_bracket)
                drag.append(dD_high)
                Reynolds.append(Re_high)
                n = n + 1
                print("Element", n, "capped, chord:", top_bracket, "m, AoA:", a_high, "deg, dT:", dT_high, "N")
                drag_tot = sum(drag)
                continue
            
            

            Re_low = bottom_bracket * V_rel/ nu
            Re_high = top_bracket * V_rel/ nu
            Re_mid = mid_bracket * V_rel/ nu

            a_low, Cl_low, Cd_low = interp_airfoil_polar(Re_low, airfoil)
            a_high, Cl_high, Cd_high = interp_airfoil_polar(Re_high, airfoil)
            a_mid, Cl_mid, Cd_mid = interp_airfoil_polar(Re_mid, airfoil)



            dT_low = 0.5 * rho * V_rel**2 * bottom_bracket * Cl_low * dr*math.cos(V_angle) - 0.5 * rho * V_rel**2 * bottom_bracket * Cd_low * dr*math.sin(V_angle)
            dT_high = 0.5 * rho * V_rel**2 * top_bracket * Cl_high * dr*math.cos(V_angle) - 0.5 * rho * V_rel**2 * top_bracket * Cd_high * dr*math.sin(V_angle)
            dT_mid = 0.5 * rho * V_rel**2 * mid_bracket * Cl_mid * dr*math.cos(V_angle) - 0.5 * rho * V_rel**2 * mid_bracket * Cd_mid * dr*math.sin(V_angle)
        

            if (dT - dT_low) * (dT - dT_mid) < 0: #Lower bracket has root
                bottom_bracket = bottom_bracket
                top_bracket = mid_bracket
                mid_bracket = (top_bracket + bottom_bracket)/2
                 

            elif (dT - dT_mid)*(dT - dT_high) < 0: #Higher bracket has root
                 bottom_bracket = mid_bracket
                 top_bracket = top_bracket
                 mid_bracket = (top_bracket + bottom_bracket)/2

            elif (dT - dT_low) == 0 or (dT - dT_high) == 0: #EXACT SOLUTION 
                thrust.append(dT)
                AoA.append(a_mid)
                chords.append(mid_bracket)
                drag.append(0.5 * rho * V_rel**2 * mid_bracket * Cd_mid * dr)
                Reynolds.append(Re_mid)
                n = n + 1
            
            if abs(dT-dT_mid) < exit_step*dT: #LOCAL EXIT CRITERIA (move to next step)
                thrust.append(dT)
                AoA.append(a_mid)
                chords.append(mid_bracket)
                drag.append(0.5 * rho * V_rel**2 * mid_bracket * Cd_mid * dr)
                Reynolds.append(Re_mid)
                n = n + 1
                print("Element", n, "solved, chord:", mid_bracket, "m, AoA:", a_mid, "deg, dT:", dT, "N")

                #reset brackets for next eelment
                #################################
                bottom_bracket = bottom_og
                top_bracket = top_og
                mid_bracket = (top_bracket + bottom_bracket)/2
                
            
            drag_tot = sum(drag)

        return chords, AoA, Reynolds, thrust, drag, drag_tot

def geometric_pitch_distribution(v_relative_angle, AoA):
        """
        Geometric pitch distribution calculation.
        Polarpicker outputs AoA in degrees and v_relative_angle in radians.
        Convert AoA to radians for calculation.
        """
        n = 0
        geo_pitch = [] #list of geometric pitch angles

        while n != len(v_relative_angle):
            alpha_rad = AoA[n] * math.pi / 180.0
            beta = v_relative_angle[n] + alpha_rad
            geo_pitch.append(beta)
            n = n + 1
        return geo_pitch
        
##########
##INPUTS##
##########

outer_d = 0.1 #diameter of prop [m]
mass = 0.13 #mass of quad [kg]
hub_d = 0.07 #hub diameter [m]
blades = 2 #number of blades
sections = 10 #number of radial sections we discretize the blade into
motor_rpm = 5000 #motor speed [rpm]
bottom_bracket = 0.00001 #min chord [m]
top_bracket = 1 #max chord [m]
exit_step = 0.001 #step size for exit condition in solver as percentage of chord [%]
y = 1 # thrust distribution: "uniform", "ramp", 0, or 1
airfoil = "s8025" # airfoil name used for the whole blade



##########################
Vp, F_prop = inflow(outer_d, mass)
print("Inflow velocity:", Vp, "m/s")
print("Thrust per motor:", F_prop, "N")

F_blade, dT_list = thrust_distribution(outer_d, hub_d, F_prop, blades, sections, y)
print("Thrust per blade:", F_blade, "N")
print("Thrust distribution mode:", y)
print("Airfoil:", airfoil)
print("Element thrusts:", dT_list)
print("Sum check:", sum(dT_list), "N")


v_relative_module, v_relative_angle = relative_velocity(motor_rpm, outer_d, hub_d, sections, Vp)

print("Relative velocity magnitudes (m/s):", v_relative_module)
print("Relative flow angles phi (rad):", v_relative_angle)

chords, AoA, Reynolds, thrust, drag, drag_tot = polarpicker(top_bracket, bottom_bracket, dT_list, v_relative_module, v_relative_angle, exit_step, outer_d, hub_d, sections, airfoil=airfoil)
print("Chord distribution (m):", chords)
print("Angle of attack distribution (deg):", AoA)
print("Reynolds distribution:", Reynolds)
print("Thrust distribution (N):", thrust)
print("Drag distribution (N):", drag)
print("Total drag (N):", drag_tot)

geo_pitch = geometric_pitch_distribution(v_relative_angle, AoA)
print("Geometric pitch distribution (rad):", geo_pitch)
