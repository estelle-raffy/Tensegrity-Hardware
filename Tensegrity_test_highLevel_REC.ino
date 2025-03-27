# 3-level system (mechanical, module-level, system-level)
# hard-coded lists for middle level 'modules' (triangles) representation
# Module 3 ONLY gets info about higher goal (x_coordinate target in space, 'target_position')
# 'current_position' as updated body_4 x_coordinate (rightmost circle, module 3)
# Module 1 and 2 only get local information (local_demand + neighbour_condition) in LENGTH 
# 'local_demand', corresponds to total length of a module's springs (sum passive springs + actuator) 
# 'neighbour_condition' as global variable 1 or -1...
#...(higher constraint, modules try to get different or similar springs length, respectively)

# Being_selfish function controls neighbour weights of modules 1 & 2 ==> if error > frustration level, play selfish (local demand only)
# Module 3 also can turn selfishness on/off depending on global frustration

import pygame
import pymunk
import pymunk.pygame_util
import math
import os
import time
import openpyxl
from openpyxl import Workbook

pygame.init()

#Make a pygame window
WIDTH, HEIGHT = 900, 590
window = pygame.display.set_mode((WIDTH, HEIGHT))


# list of colors
GREEN = (0, 255, 0),  # Green
BLUE = (0, 0, 255),  # Blue
YELLOW = (255, 255, 0),  # Yellow
MAGENTA = (255, 0, 255)   # Magenta

#draw function
def draw(space, window, draw_options):
    window.fill((255, 255, 255))
    space.debug_draw(draw_options)# may override the colors 
    pygame.display.flip()

#Boundaries
def create_boundaries(space, width, height):
    rects = [
        [(width/2, height - 10), (width, 20)],  # Floor
        [(width/2, 10), (width, 20)],  # Ceiling
        [(10, height/2), (20, height)],  # Left wall
        [(width - 10, height/2), (20, height)]  # Right wall
    ]

    for pos, size in rects:
        body = pymunk.Body(body_type=pymunk.Body.STATIC)
        body.position = pos
        shape = pymunk.Poly.create_box(body, size)
        shape.elasticity = 0
        shape.friction = 0.1
        space.add(body, shape)

def create_system (space, mass, damping, stiffness):

    global module1, module2, module3
    global passive_springs
    global actuators
    global bodies

    module1 = []
    module2 = []
    module3 = []
    passive_springs = []
    actuators = []
    bodies = []
    
     # Screen dimensions and floor offset
    SCREEN_WIDTH = 900
    SCREEN_HEIGHT = 590
    FLOOR_OFFSET = 40

    # Bodies and circles for the first triangle (Module 1)
    body1 = pymunk.Body(body_type=pymunk.Body.STATIC)
    body1_x_position = SCREEN_WIDTH / 2 - 350 #100
    body2_y_position = SCREEN_HEIGHT - FLOOR_OFFSET #550
    body1.position = (body1_x_position, body2_y_position)
    circle1 = pymunk.Circle(body1, body_diameter)
    #circle1.mass = mass
    circle1.friction = friction_bodies
    circle1.elasticity = elasticity_bodies
    module1.append(body1)

    body2 = pymunk.Body()
    body2.position = (SCREEN_WIDTH / 2 - 250 , SCREEN_HEIGHT - FLOOR_OFFSET)# 200, 550
    circle2 = pymunk.Circle(body2, body_diameter)
    circle2.mass = mass
    circle2.friction = friction_bodies
    circle2.elasticity = elasticity_bodies
    module1.append(body2)
    module2.append(body2)
    module3.append(body2)

    body3 = pymunk.Body()
    body3.position = (SCREEN_WIDTH / 2 - 300, SCREEN_HEIGHT - FLOOR_OFFSET - 86.6)# 150, 463.3
    circle3 = pymunk.Circle(body3, body_diameter)
    circle3.mass = mass
    circle3.friction = friction_bodies
    circle3.elasticity = elasticity_bodies
    module1.append(body3)
    module2.append(body3)

    # Body and circle for the second triangle (Module 2)
    body5 = pymunk.Body()
    body5.position = (SCREEN_WIDTH / 2 - 200 , SCREEN_HEIGHT - FLOOR_OFFSET - 86.6) #250, 463.3
    circle5 = pymunk.Circle(body5, body_diameter)
    circle5.mass = mass
    circle5.friction = friction_bodies
    circle5.elasticity = elasticity_bodies
    module2.append(body5)
    module3.append(body5)

    # Body and circle for the third triangle (Module 3)
    body4 = pymunk.Body()
    body4_x_position =  SCREEN_WIDTH / 2 - 150 # x_pos 300
    body4_y_position = SCREEN_HEIGHT - FLOOR_OFFSET # y_pos 550
    body4.position = ( body4_x_position,body4_y_position)
    circle4 = pymunk.Circle(body4, body_diameter)
    circle4.mass = mass
    circle4.friction = friction_bodies
    circle4.elasticity = elasticity_bodies
    module3.append(body4)

    print("Body 4 starting position: ")
    print(body4_x_position, body4_y_position)

    # Anchor points in the middle
    anch1 = (0, 0)
    anch2 = (0, 0)
    anch3 = (0, 0)
    anch4 = (0, 0)
    anch5 = (0, 0)

    # Spring joints for the first triangle (Module 1)
    joint1 = pymunk.constraints.DampedSpring(body1, body3, anch1, anch3, rest_length=100, stiffness=stiffness, damping=damping)
    module1.append(joint1)
    passive_springs.append(joint1)
    joint2 = pymunk.constraints.DampedSpring(body3, body2, anch3, anch2, rest_length=100, stiffness=stiffness, damping=damping)
    module1.append(joint2)
    module2.append(joint2)
    passive_springs.append(joint2)
    actuator1 = pymunk.constraints.DampedSpring(body1, body2, anch1, anch2, rest_length=100, stiffness=stiffness, damping=damping)
    module1.append(actuator1)
    actuators.append(actuator1)

    # Adding the new joints and actuators for the second triangle (Module 2)
    joint3 = pymunk.constraints.DampedSpring(body2, body5, anch3, anch5, rest_length=100, stiffness=stiffness, damping=damping)
    module2.append(joint3)
    module3.append(joint3)
    passive_springs.append(joint3)
    actuator2 = pymunk.constraints.DampedSpring(body3, body5, anch2, anch5, rest_length=100, stiffness=stiffness, damping=damping)
    module2.append(actuator2)
    actuators.append(actuator2)

    # Adding the new joints and actuators for the third triangle (Module 3)
    joint4 = pymunk.constraints.DampedSpring(body5, body4, anch5, anch4, rest_length=100, stiffness=stiffness, damping=damping)
    module3.append(joint4)
    passive_springs.append(joint4)
    actuator3 = pymunk.constraints.DampedSpring(body2, body4, anch2, anch4, rest_length=100, stiffness=stiffness, damping=damping)
    module3.append(actuator3)
    actuators.append(actuator3)

    #For easier indexing 
    #module1 = body1, body2, body3, joint1, joint2, actuator1
    #module2 = body2, body3, body5, joint2, joint3, actuator2
    #module3 = body2, body5, body4, joint3, joint4, actuator3


    #passive_springs = joint1, joint2, joint3, joint4
    #actuators = actuator1, actuator2, actuator3

    # Add all bodies, circles, and joints to the space
    space.add(body1, circle1, body2, circle2, body3, circle3, body4, circle4, body5, circle5, 
              joint1, joint2, actuator1, joint3, joint4, actuator2, actuator3)

def create_fixed_target(space, global_target_x, global_target_y):
    body = pymunk.Body(body_type=pymunk.Body.STATIC)
    body_x_position = global_target_x # can change position (default 700)
    body_y_position = global_target_y
    body.position = (body_x_position, body_y_position)
    target = pymunk.Circle(body, 20)
    #target.color = GREEN
    space.add(body, target)

    return body_x_position, body_y_position

def test_stable_state(step): #increment all actuators length by 1 and see if stable at target location
    
    # Always retrieve positions before using 
    body4_current_pos_x, body4_current_pos_y = get_current_position_body1()
    body4_current_pos_x, body4_current_pos_y = get_current_position_body2()
    body4_current_pos_x, body4_current_pos_y = get_current_position_body3()
    body4_current_pos_x, body4_current_pos_y = get_current_position_body4()
    body4_current_pos_x, body4_current_pos_y = get_current_position_body4()

    # Elongate all actuators by step
    actuator1 = actuators[0]
    actuator2 = actuators[1]
    actuator3 = actuators[2]
    
    actuator1.rest_length += step
    actuator2.rest_length += step
    actuator3.rest_length += step

    # Get new position & convert to MatLab values 

    body1_x, body1_y = get_current_position_body1()
    body1_matlab_pos_x = body1_x - 300
    body1_matlab_pos_y = (body1_y  - 550) * (-1)
    #print(body1_matlab_pos_x, body1_matlab_pos_y)

    body2_x, body2_y = get_current_position_body2()
    body2_matlab_pos_x = body2_x - 300
    body2_matlab_pos_y = (body2_y  - 550) * (-1)

    body3_x, body3_y = get_current_position_body3()
    body3_matlab_pos_x = body3_x - 300
    body3_matlab_pos_y = (body3_y  - 550) * (-1)

    body4_x, body4_y = get_current_position_body4()
    body4_matlab_pos_x = body4_x - 300
    body4_matlab_pos_y = (body4_y  - 550) * (-1)
    #print(body4_matlab_pos_x, body4_matlab_pos_y)

    body5_x, body5_y = get_current_position_body5()
    body5_matlab_pos_x = body5_x - 300
    body5_matlab_pos_y = (body5_y  - 550) * (-1)

    return body1_matlab_pos_x, body1_matlab_pos_y, body2_matlab_pos_x, body2_matlab_pos_y, body3_matlab_pos_x, body3_matlab_pos_y, body4_matlab_pos_x, body4_matlab_pos_y, body5_matlab_pos_x, body5_matlab_pos_y
    
def get_current_position_body4(): # higher-goal 

    body4_position = module3[2]
    body4_current_pos_x = body4_position.position.x
    body4_current_pos_y = body4_position.position.y
    
    return body4_current_pos_x, body4_current_pos_y

def get_current_position_body1():

    body1_position = module1[0]
    body1_current_pos_x = body1_position.position.x
    body1_current_pos_y = body1_position.position.y

    body1_matlab_pos_x = body1_current_pos_x - 300
    body1_matlab_pos_y = (body1_current_pos_y  - 550) * (-1)
    
    return body1_matlab_pos_x, body1_matlab_pos_y

def get_current_position_body2():

    body2_position = module1[1]
    body2_current_pos_x = body2_position.position.x
    body2_current_pos_y = body2_position.position.y
    
    body2_matlab_pos_x = body2_current_pos_x - 300
    body2_matlab_pos_y = (body2_current_pos_y  - 550) * (-1)
    
    return body2_matlab_pos_x, body2_matlab_pos_y

def get_current_position_body3():

    body3_position = module1[2]
    body3_current_pos_x = body3_position.position.x
    body3_current_pos_y = body3_position.position.y

    body3_matlab_pos_x = body3_current_pos_x - 300
    body3_matlab_pos_y = (body3_current_pos_y  - 550) * (-1)
    
    return body3_matlab_pos_x, body3_matlab_pos_y

def get_current_position_body5():

    body5_position = module3[1]
    body5_current_pos_x = body5_position.position.x
    body5_current_pos_y = body5_position.position.y
    
    body5_matlab_pos_x = body5_current_pos_x - 300
    body5_matlab_pos_y = (body5_current_pos_y  - 550) * (-1)
    
    return body5_matlab_pos_x, body5_matlab_pos_y
    
def m1_bigger_than_m2(step):

    spring1 = passive_springs[0]
    spring2 = passive_springs[1]
    actuator1 = actuator[0]

    number_steps = 50

    for i in range (number_steps):
        spring1.rest_length += step 
        spring2.rest_length += step
        actuator1.rest_length += step

    print('M1 increased by ',  number_steps, ' steps!')
    
    rest_length1 = spring1.rest_length
    print("J1 rest length updated", rest_length1)
    rest_length2 = spring2.rest_length
    print("J2 rest length updated", rest_length2)
    rest_length = actuator1.rest_length
    print("A1 rest length updated", rest_length)
    
def m2_bigger_than_m1(step):
    
    spring2 = passive_springs[1]
    spring3 = passive_springs[2]
    actuator2 = actuators[1]

    number_steps = 40
    
    for i in range (number_steps):
        #spring2.rest_length += step #shared with M1 so turned off if want M2 > M1
        spring3.rest_length += step
        actuator2.rest_length += step

    print('M2 increased by ',  number_steps, ' steps!')
    
    rest_length2 = spring2.rest_length
    print("J2 rest length updated", rest_length2)
    rest_length3 = spring3.rest_length
    print("J3 rest length updated", rest_length3)
    rest_length = actuator2.rest_length
    print("A2 rest length updated", rest_length)

def get_current_length(body1, body2):
    current_length = math.sqrt((body2.position.x - body1.position.x)**2 + (body2.position.y - body1.position.y)**2)
    return current_length

def get_length_module1():
    
    spring1 = passive_springs[0]
    length1 = get_current_length(spring1.a , spring1.b)

    spring2 = passive_springs[1]
    length2 = get_current_length(spring2.a , spring2.b)

    actuator1 = actuators[0]
    length  = actuator1.rest_length
    
    total_length = (length1 + length2 + length)

    return total_length

def get_length_module2():
    
    spring2 = passive_springs[1]
    length2 = get_current_length(spring2.a , spring2.b)

    spring3 = passive_springs[2]
    length3 = get_current_length(spring3.a , spring3.b)

    actuator2 = actuators[1]
    length  = actuator2.rest_length
    
    total_length = (length2 + length3+ length)

    return total_length

def get_length_module3():
    
    spring3 = passive_springs[2]
    length3 = get_current_length(spring3.a , spring3.b)

    spring4 = passive_springs[3]
    length4 = get_current_length(spring4.a , spring4.b)

    actuator3 = actuators[2]
    length  = actuator3.rest_length
    
    total_length = (length3 + length4+ length)

    return total_length

def being_selfish(beingSelfishM1, beingSelfishM2, M1_new_local_err, M2_new_local_err, local_threshold, 
                  M1_local_integrated_error, M2_local_integrated_error, M1_local_frustration, M2_local_frustration,
                  local_frustration_threshold, local_recovery_threshold, lambda_local, weight_reset):

    print("CHECKING VALUE ", local_frustration_threshold)
    
    global Wb_M1
    global Wb_M2
    
    # WORK OUT FOR M1
    M1_local_integrated_error = M1_local_integrated_error + (M1_new_local_err - (lambda_local * M1_local_integrated_error)) * 1
    M1_local_frustration += M1_local_integrated_error

    print('M1 frustration levels: ', M1_local_frustration)

    if beingSelfishM1 == 1: 
        if abs(M1_local_frustration) < local_recovery_threshold:
            Wb_M1 = weight_reset # should reset there or let it 0?
            print('M1 plays collective below recovery threshold, Wb_M1: ', Wb_M1)
        elif abs(M1_local_frustration) < local_frustration_threshold:
            Wb_M1 = weight_reset
            print('M1 plays collective below frustration threshold, Wb_M1: ', Wb_M1)
        elif abs(M1_local_frustration) > local_frustration_threshold:
            Wb_M1 = 0
            M1_local_integrated_error = 0
            M1_local_frustration = 0
            print('M1 plays selfish, Wb_M1: ', Wb_M1)
            print('FRUSTRATION THRESHOLD REACHED: Reset frustration for M1')
    else:
        M1_local_integrated_error = 0
        M1_local_frustration = 0
        print('Being_selfish is not active for M1')

    # WORK OUT FOR M2

    M2_local_integrated_error = M2_local_integrated_error + (M2_new_local_err - (lambda_local * M2_local_integrated_error)) * 1
    M2_local_frustration += M2_local_integrated_error

    print('M2 frustration levels: ', M2_local_frustration)

    if beingSelfishM2 == 1: 
        if abs(M2_local_frustration) < local_recovery_threshold:
            Wb_M2 = weight_reset
            print('M2 plays collective below recovery threshold, Wb_M2: ', Wb_M2)
        elif abs(M2_local_frustration) < local_frustration_threshold:
            Wb_M2 = weight_reset
            print('M2 plays collective below frustration threshold, Wb_M2: ', Wb_M2)
        elif abs(M2_local_frustration) > local_frustration_threshold:
            Wb_M2 = 0
            M2_local_integrated_error = 0
            M2_local_frustration = 0
            print('M2 plays selfish, Wb_M2: ', Wb_M2)
            print('FRUSTRATION THRESHOLD REACHED: Reset frustration for M2')
    else:
        M2_local_integrated_error = 0
        M2_local_frustration = 0
        print('Being_selfish is not active for M2')

    return M1_local_frustration, M2_local_frustration, beingSelfishM1, beingSelfishM2, Wb_M1, Wb_M2
  
    
def local_err_red_module1 (local_demand, local_threshold, neigh_threshold_converge, neigh_threshold_diverge, step, Wa, Wb_M1, Wn_M1): 

    # Update M1 and M2 total lengths before using them
    M1_total_length = get_length_module1()
    M2_total_length = get_length_module2()
    #M3_total_length = get_length_module3()

    print('M1 total length before step: ', M1_total_length)
    print('M2 total length before step: ', M2_total_length)
    #print('M3 total length before step: ', M3_total_length)

    actuation_signal_elongates = 0
    actuation_signal_shortens = 0 
    
    # Lower-level err-red
    local_err_red = local_demand - M1_total_length 
    print ('************ M1 local error before step: ',local_err_red)
    local_adj = Wa * step 

    # LOCAL ACTUATION SIGNAL
    actuator1 = module1[5]

    if abs(local_err_red) < local_threshold:
        print("--------------> M1 LOCAL DEMAND ACHIEVED")
    elif local_err_red > 0:
        actuation_signal_elongates += local_adj 
        print('M1 increases by', local_adj, ' to local')
    elif local_err_red < 0:
        actuation_signal_shortens += local_adj 
        print('M1 decreases by', local_adj,' to local')
    else :
        print('M1 LOCAL ISSUE DETECTED')

    print('actuation_signal_elongates: ', actuation_signal_elongates)
    print('actuation_signal_shortens: ', actuation_signal_shortens)
    
    # NEIGHBOUR ACTUATION SIGNAL 
    neigh_diff = M1_total_length - M2_total_length
    #neigh_diff = M1_total_length - M3_total_length
    print ('************* M1 neighbour difference: ',neigh_diff)
    neigh_adj = Wb_M1 * step

    if Wn_M1 == -1: # converge
        print("inif-block, CHECKING WN_M1 ", Wn_M1)
        if abs(neigh_diff) < neigh_threshold_converge:
            print('M1 NEIGHBOUR ACHIEVED')
        elif neigh_diff > 0: #bigger than M2
            actuation_signal_shortens += neigh_adj
            print('M1 decreases by ', neigh_adj, ' to converge')
        elif neigh_diff < 0: # smaller than M2
            actuation_signal_elongates += neigh_adj
            print('M1 increase by ', neigh_adj, ' to converge')
    elif Wn_M1 == 1: # diverge, but if neigh_diff = 0, doesn't move... 
        if abs(neigh_diff) < neigh_threshold_diverge:    
            if neigh_diff > 0 : #bigger than M2
                actuation_signal_elongates += neigh_adj
                print('M1 increases by ', neigh_adj, ' to diverge')
            if neigh_diff < 0 : # smaller than M2 
                actuation_signal_shortens += neigh_adj
                print('M1 decrease by ', neigh_adj, ' to diverge')
        elif abs(neigh_diff) > neigh_threshold_diverge: 
            print('M1 NEIGH GONE TOO FAR, NEIGH STOPS')
        else:
            print('Incorrect neigh_diff value')  
    else:
        print('M1 NEIGHBOUR ISSUE DETECTED') 
       

    ## WORKING OUT SINGLE ACTUATION SIGNAL
    actuation_final = actuation_signal_elongates - actuation_signal_shortens
    if actuation_final > 0:
        actuator1.rest_length += actuation_final
    elif actuation_final < 0 :
        actuator1.rest_length -= abs(actuation_final)
    else:
        print('M1 STOPS')

    print('M1 actuation final: ', actuation_final)
    M1_new_total_length = get_length_module1()
    print('M1 total length after step: ', M1_new_total_length)
    new_local_err_red = local_demand - M1_new_total_length 
    print ('######## M1 local error after step: ',new_local_err_red)

    return M1_total_length, local_err_red, neigh_diff, actuation_final, M1_new_total_length, new_local_err_red 

def local_err_red_module2 (local_demand, local_threshold, neigh_threshold_converge, neigh_threshold_diverge, step, Wa, Wb_M2, Wn_M2):  

    # Update M1 & M2 total lengths before using them
    M3_total_length = get_length_module3()
    M2_total_length = get_length_module2()
    #M1_total_length = get_length_module1()

    print('M2 total length before step: ', M2_total_length)
    print('M3 total length before step: ', M3_total_length)
    #print('M1 total length before step: ', M1_total_length)

    actuation_signal_elongates = 0
    actuation_signal_shortens = 0 

    # Lower-level err-red
    local_err_red = local_demand - M2_total_length 
    print ('***********M2 local error: ',local_err_red)
    local_adj = Wa * step

    #LOCAL ACTUATION SIGNAL
    actuator2 = module2[5]
    if abs(local_err_red) < local_threshold:
        print('--------------> M2 LOCAL DEMAND ACHIEVED')
    elif local_err_red > 0:
        actuation_signal_elongates += local_adj
        print('M2 increases by ', local_adj, ' to local')
    elif local_err_red < 0:
        actuation_signal_shortens += local_adj
        print('M2 decreases by', local_adj, ' to local')
    else :
            print('M2 LOCAL ISSUE DETECTED')

    # NEIGHBOUR ACTUATION SIGNAL  
    neigh_diff = M2_total_length - M3_total_length
    #neigh_diff = M2_total_length - M1_total_length
    print ('*********** M2 neighbour difference: ',neigh_diff)
    neigh_adj = Wb_M2 * step

    if Wn_M2 == -1:
        if abs(neigh_diff) < neigh_threshold_converge:
            print('M2 NEIGHBOUR ACHIEVED')
        elif neigh_diff > 0: #bigger than M3
            actuation_signal_shortens += neigh_adj
            print('M2 decreases by ', neigh_adj, ' to converge')
        elif neigh_diff < 0: # smaller than M3
            actuation_signal_elongates += neigh_adj
            print('M2 increase by ', neigh_adj, ' to converge')
    elif Wn_M2 == 1:
        if abs(neigh_diff) < neigh_threshold_diverge:
            if neigh_diff > 0 : #bigger than M3
                actuation_signal_elongates += neigh_adj
                print('M2 increases by ', neigh_adj, ' to diverge')
            if neigh_diff < 0 : #smaller than M3
                actuation_signal_shortens += neigh_adj
                print('M2 decrease by ', neigh_adj, ' to diverge')
        elif abs(neigh_diff) < neigh_threshold_diverge:
            print('M2 NEIGH GONE TOO FAR, NEIGH STOPS')
        else:
            print('Incorrect neigh_diff value')
    else:
        print('M2 NEIGHBOUR ISSUE DETECTED') 


    ## WORKING OUT SINGLE ACTUATION SIGNAL
    actuation_final = actuation_signal_elongates - actuation_signal_shortens
    if actuation_final > 0:
        actuator2.rest_length += actuation_final
    elif actuation_final < 0:
        actuator2.rest_length -= abs(actuation_final)
    else:
        print('M2 STOPS')

    print('M2 actuation final: ', actuation_final)
    M2_new_total_length = get_length_module2()
    print('M2 total length after step: ', M2_new_total_length)
    new_local_err_red = local_demand - M2_new_total_length 
    print ('######## M2 local error after step: ',new_local_err_red)

    return M2_total_length, local_err_red, neigh_diff, actuation_final, M2_new_total_length, new_local_err_red

def global_err_red_module3 (global_target_x, global_target_y, global_threshold, global_integrated_error, global_frustration,
                            global_frustration_threshold, global_recovery_threshold, lambda_global, beingSelfishM1, beingSelfishM2, step, Wc, weight_reset): 

    global Wb_M1
    global Wb_M2
    
    #Update positions
    body4_current_pos_x, body4_current_pos_y = get_current_position_body4()
    print ('B4 Starting position', body4_current_pos_x, body4_current_pos_y)
    
    # Equation
    error = math.sqrt((global_target_x - body4_current_pos_x)**2 + (global_target_y - body4_current_pos_y)**2)# maybe should keep Wc for ajustment in moving like M1 & M2
    print("----------------------->> global error: ", error)
    
    # Working out how to move
    actuator3 = actuators[2]
    global_adj = Wc * step

    if abs(error) < global_threshold : 
        print ('GLOBAL TARGET SUCCESS!')
        global_integrated_error = 0
        global_frustration = 0
    elif error > 0:
        actuator3.rest_length += global_adj
        print ('Moving up to global, new actuator length: ', actuator3.rest_length)
    elif error < 0:
        actuator3.rest_length -= global_adj
        print ('Moving down to global')
    else :
        print('GLOBAL ISSUE DETECTED')

    #MODULATING SELFISHNESS
    global_integrated_error = global_integrated_error + (error - (lambda_global * global_integrated_error)) * 1
    global_frustration += global_integrated_error
    print("Global frustration level: ", global_frustration)
    print(global_recovery_threshold)
    print(global_frustration_threshold)

    if abs(global_frustration) < global_recovery_threshold:
        print("Below global recovery")
        if beingSelfishM1 == 0 or beingSelfishM2 == 0:
            print("SELFISHNESS ALLOWED")
        beingSelfishM1 = 1 # make sure it is on/back on anyway
        beingSelfishM2 = 1
    elif abs(global_frustration) < global_frustration_threshold:
        print("Global frustration level still acceptable")
    elif abs(global_frustration) > global_frustration_threshold:
        if beingSelfishM1 == 1 or beingSelfishM2 == 1:
            print('ABOVE FRUSTRATION THRESHOLD ==> OVERRULING SELFISHNESS!')
        Wb_M1 = weight_reset
        Wb_M2 = weight_reset
        print('OVERRULING, Wb_M1: ', Wb_M1)
        print('OVERRULING, Wb_M2: ', Wb_M2)
        beingSelfishM1 = 0
        beingSelfishM2 = 0
        global_integrated_error = 0
        global_frustration = 0
        print('Frustration RESET')
    else:
        print('Error with global modulation')

    print('global weights', Wb_M1, Wb_M2)

    Matlab_body4_pos_x = body4_current_pos_x - 300
    Matlab_body4_pos_y = (body4_current_pos_y  - 550) * (-1)

    #print ('MATLAB: ', Matlab_body4_pos_x, Matlab_body4_pos_y)
    
    return Matlab_body4_pos_x, Matlab_body4_pos_y, error, global_frustration, beingSelfishM1, beingSelfishM2, Wb_M1, Wb_M2 

# Main event loop
def run(window, width, height, simulation_duration):

    # Excel business
    script_dir = os.path.dirname(os.path.abspath(__file__))
    excel_file_path = os.path.join(script_dir, '3_triangles_LEVELS_HigherSystem.xlsx')
        
    run = True
    clock = pygame.time.Clock()
    fps = 100 # could help stability, longer simulation duration 
    dt = 1 / fps
    delay = 2000

    # Make pymunk space
    space = pymunk.Space()
    space.gravity = (0, 981) #981

    # Physical parameters
    global body_diameter 
    body_diameter = 21 #mimicking struts/cords ratio?
    global friction_bodies
    friction_bodies = 1 # rubber cap, considering soft Rubber-like Materials e.g., Agilus30: dymamic coeff 0.8-1.2
    global elasticity_bodies
    elasticity_bodies = 0 # considered rigids
    mass = 0.38 #circles/bodies 
    stiffness = 100 #100 for cords 
    damping = 0.1

    # LOWER-LEVEL Parameters 
    step = 1 # default = 1
    local_demand = 400 #target length (sum) 310
    local_threshold = 10
    neigh_threshold_converge = 10
    neigh_threshold_diverge = 100
    M1_local_frustration = 0
    M2_local_frustration = 0
    M1_local_integrated_error = 0
    M2_local_integrated_error = 0
    local_frustration_threshold = local_demand * 2 #threshold always take absolute so no need for min
    local_recovery_threshold = local_demand * 1.5
    lambda_local = 0.2
    beingSelfishM1 = 1
    beingSelfishM2 = 1 #start with lower-level control
  

    # HIGHER-LEVEL Parameters 
    Wa = 0.33 # influence of local demand
    Wb_M1 = 1
    Wb_M2 = 1 # influence of neighbour condition
    final_Wb_M1 = 1 # to track global adjustments 
    final_Wb_M2 = 1
    weight_reset = 1
    Wc = 0.33 # influence of global demand
    Wn_M1 = 1 #1 = diff NRG, -1 = same NRG
    Wn_M2 = -1
    global_integrated_error = 0
    global_frustration = 0
    lambda_global = 0.8

    # Excel business
    length_log = []

    # Call the functions ==================================================
    create_boundaries(space, width, height)
    create_system(space, mass, damping, stiffness)
 
    # Set higher-level goal
    global_target_x = 500 # 500m; 100 symmetry behind 
    global_target_y = 450 # 550 floor, 450 AIR 
    global_threshold = 45 # accounting for distance from edges of ball to target centre
    create_fixed_target(space, global_target_x, global_target_y)
    
    global_frustration_threshold = global_threshold * 100
    global_recovery_threshold = global_threshold * 80

    # Save modified target coordinates for MATLAB (making up for weird simulation boundaries and coordinates...)
    target_x = global_target_x - 300
    target_y = (global_target_y - 550) * (-1)
    global_threshold_x = target_x - global_threshold
    global_threshold_y = target_y - global_threshold
 
    draw_options = pymunk.pygame_util.DrawOptions(window)

    # create difference in energy between modules
    #m2_bigger_than_m1(step)

    # start simulation
    start_time = time.time()
    timestep = 0
    delay_to_wait = 0 #s
    error_global = 0
    local_error1 = 0
    local_error2 = 0
    global_error3 = 0
    
    # Simulation loop ****************************************************
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                break

        current_time = time.time() - start_time
                
        if current_time > delay_to_wait:
            print('========================================================> Timestep ', timestep)
            print("Local_demand is: ", local_demand)
            print('Global target is: ', global_target_x, global_target_y)

            get_current_position_body1()

            #Activate M1 
            M1_total_length, M1_local_err, M1_neigh_diff, M1_actuation_final, M1_new_total_length, M1_new_local_err = local_err_red_module1(local_demand, local_threshold, neigh_threshold_converge, neigh_threshold_diverge, step, Wa, final_Wb_M1, Wn_M1)
           
            #Activate M2
            M2_total_length, M2_local_err, M2_neigh_diff, M2_actuation_final, M2_new_total_length, M2_new_local_err = local_err_red_module2(local_demand, local_threshold, neigh_threshold_converge, neigh_threshold_diverge, step, Wa, final_Wb_M2, Wn_M2) 

            #Decide on playing collective or selfish
            M1_local_frustration, M2_local_frustration, beingSelfishM1, beingSelfishM2, local_Wb_M1, local_Wb_M2 = being_selfish(beingSelfishM1, beingSelfishM2, M1_new_local_err, M2_new_local_err, local_threshold, M1_local_integrated_error, M2_local_integrated_error,  M1_local_frustration, M2_local_frustration,
                  local_frustration_threshold, local_recovery_threshold, lambda_local, weight_reset)
            
            #Activate M3
            body4_pos_x, body4_pos_y, global_error, global_frustration, beingSelfishM1, beingSelfishM2, final_Wb_M1, final_Wb_M2 = global_err_red_module3(global_target_x, global_target_y, global_threshold, global_integrated_error, global_frustration,
                            global_frustration_threshold, global_recovery_threshold, lambda_global, beingSelfishM1, beingSelfishM2, step, Wc, weight_reset)

            print('Final Wb_M1 is ', final_Wb_M1)
            print('Final Wb_M2 is ', final_Wb_M2)

            
            # Get other bodies positions

            body1_pos_x, body1_pos_y = get_current_position_body1()
            body2_pos_x, body2_pos_y = get_current_position_body2()
            body3_pos_x, body3_pos_y = get_current_position_body3()
            body5_pos_x, body5_pos_y = get_current_position_body5()
            
            timestep += 1
            #time.sleep(10)
            
        draw(space, window, draw_options)
        space.step(dt)

        #Excel business 
        length_log.append([current_time, target_x, target_y, global_threshold_x, global_threshold_y, body1_pos_x, body1_pos_y,
                           body2_pos_x, body2_pos_y, body3_pos_x, body3_pos_y,
                           body4_pos_x, body4_pos_y, body5_pos_x, body5_pos_y,
                           local_demand, M1_total_length, M1_local_err, M1_neigh_diff, M1_actuation_final, M1_new_total_length, M1_new_local_err,
                           M1_local_frustration, M2_total_length, M2_local_err, M2_neigh_diff, M2_actuation_final,
                           M2_new_total_length, M2_new_local_err, M2_local_frustration, local_Wb_M1, local_Wb_M2, global_target_x, global_target_y,
                           global_error, global_frustration, final_Wb_M1, final_Wb_M2])

        if current_time >= simulation_duration:
            run = False

        clock.tick(fps)

    # create ad save workbook
    wb = Workbook()
    ws = wb.active
    ws.append(['current_time', 'target_x', 'target_y', 'global_threshold_x', 'global_threshold_y', 'body1_pos_x', 'body1_pos_y',
                           'body2_pos_x', 'body2_pos_y', 'body3_pos_x', 'body3_pos_y',
                           'body4_pos_x', 'body4_pos_y', 'body5_pos_x', 'body5_pos_y', 'local_demand',
                           'M1_total_length', 'M1_local_err', 'M1_neigh_diff', 'M1_actuation_final', 'M1_new_total_length', 'M1_new_local_err',
                           'M1_local_frustration', 'M2_total_length', 'M2_local_err', 'M2_neigh_diff', 'M2_actuation_final',
                           'M2_new_total_length', 'M2_new_local_err', 'M2_local_frustration', 'local_Wb_M1', 'local_Wb_M2', 'global_target_x', 'global_target_y',
                           'global_error', 'global_frustration','final_Wb_M1', 'final_Wb_M2'])

    for line in length_log:
        ws.append(line)

    wb.save(excel_file_path)

    pygame.quit()

if __name__ == '__main__':
    # Specify the simulation duration in seconds
    simulation_duration = 60  #180
    pygame.display.set_caption("3_triangles_err_red_LEVELSV2_HigherSystem")

    run(window, WIDTH, HEIGHT, simulation_duration)


