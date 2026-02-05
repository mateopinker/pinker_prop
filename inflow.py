import math

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

def thrust_distribution_uniform(outer_d, hub_d, F_prop, n_blades, sections):
    """
    Uniform thrust distribution discretized into 'sections' radial elements.
    Uses a while loop (consistent with relative_velocity).

    Returns:
        F_blade : thrust per blade [N]
        dT_list : list of thrust per radial section [N]
    """
    outer_r = outer_d / 2
    root_r  = hub_d / 2
    span = outer_r - root_r
    dr = span / sections

    F_blade = F_prop / n_blades # [N]
    dT_dr = F_blade / span # [N/m]

    dT_list = []
    n = 0

    while n != sections:
        dT = dT_dr * dr # thrust in this section [N]
        dT_list.append(dT)
        n = n + 1

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

def interp_s8025_polar(Re):
    """
    Linear interpolation of the polars for the S8025 airfoil.
    For a given reynolds returns optimal angle of attack (deg),
    lift coefficient, and drag coefficient. (alpha_opt_deg, Cl_opt, Cd_opt
    NO EXTRAPOLATION BEYOND DATA RANGE.
    """

    Re0, a0, cl0, cd0 = 50000,  5.75, 0.6472, 0.02431
    Re1, a1, cl1, cd1 = 100000,  6.25, 0.7023, 0.01889
    Re2, a2, cl2, cd2 = 200000,  6.50, 0.7340, 0.01501
    Re3, a3, cl3, cd3 = 500000,  7.00, 0.7928, 0.01217
    Re4, a4, cl4, cd4 = 1000000,  6.75, 0.7713, 0.00989
    # I clamp data range (no extrapolation)
    if Re <= Re0:
        return a0, cl0, cd0
    if Re >= Re4:
        return a4, cl4, cd4

    if Re <= Re1:
        t = (Re - Re0) / (Re1 - Re0)
        return (a0 + t*(a1 - a0),cl0 + t*(cl1 - cl0),cd0 + t*(cd1 - cd0))

    if Re <= Re2:
        t = (Re - Re1) / (Re2 - Re1)
        return (a1 + t*(a2 - a1),cl1 + t*(cl2 - cl1), cd1 + t*(cd2 - cd1))

    if Re <= Re3:
        t = (Re - Re2) / (Re3 - Re2)
        return (a2 + t*(a3 - a2),cl2 + t*(cl3 - cl2),cd2 + t*(cd3 - cd2))

    t = (Re - Re3) / (Re4 - Re3)
    return (a3 + t*(a4 - a3),cl3 + t*(cl4 - cl3),cd3 + t*(cd4 - cd3))


def polarpicker(top_bracket, bottom_bracket,dT_list, v_relative_module,v_relative_angle,exit_step,outer_d, hub_d, sections,rho=1.225,nu=1.46e-5):
        """
        Chord and AoA distribution calculation.
        I compute dT=dLcos(phi) - dDsin(phi) and
        dL =  0.5 * rho *V_rel**2 * chord * Cl *dr
        dD = 0.5 * rho *V_rel**2 * chord * Cd * dr
        I use the bisection method to find the chord that satisfies the equation for each element
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




        while n != sections:
            dT = dT_list[n]
            V_rel = v_relative_module[n]
            V_angle = v_relative_angle[n]
            
            

            Re_low = bottom_bracket * V_rel/ nu
            Re_high = top_bracket * V_rel/ nu
            Re_mid = mid_bracket * V_rel/ nu

            a_low, Cl_low, Cd_low = interp_s8025_polar(Re_low)
            a_high, Cl_high, Cd_high = interp_s8025_polar(Re_high)
            a_mid, Cl_mid, Cd_mid = interp_s8025_polar(Re_mid)



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

outer_d = 0.13 #diameter of prop [m]
mass = 0.5 #mass of quad [kg]
hub_d = 0.01 #hub diameter [m]
blades = 2 #number of blades
sections = 10 #number of radial sections we discretize the blade into
motor_rpm = 20000 #motor speed [rpm]
bottom_bracket = 0.00001 #min chord [m]
top_bracket = 1 #max chord [m]
exit_step = 0.001 #step size for exit condition in solver as percentage of chord [%]


##########################
Vp, F_prop = inflow(outer_d, mass)
print("Inflow velocity:", Vp, "m/s")
print("Thrust per motor:", F_prop, "N")

F_blade, dT_list = thrust_distribution_uniform(outer_d, hub_d, F_prop, blades, sections)
print("Thrust per blade:", F_blade, "N")
print("Element thrusts:", dT_list)
print("Sum check:", sum(dT_list), "N")


v_relative_module, v_relative_angle = relative_velocity(motor_rpm, outer_d, hub_d, sections, Vp)

print("Relative velocity magnitudes (m/s):", v_relative_module)
print("Relative flow angles phi (rad):", v_relative_angle)

chords, AoA, Reynolds, thrust, drag, drag_tot = polarpicker(top_bracket, bottom_bracket, dT_list, v_relative_module, v_relative_angle, exit_step, outer_d, hub_d, sections)
print("Chord distribution (m):", chords)
print("Angle of attack distribution (deg):", AoA)
print("Reynolds distribution:", Reynolds)
print("Thrust distribution (N):", thrust)
print("Drag distribution (N):", drag)
print("Total drag (N):", drag_tot)

geo_pitch = geometric_pitch_distribution(v_relative_angle, AoA)
print("Geometric pitch distribution (rad):", geo_pitch)