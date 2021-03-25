import update_map_2
import matplotlib
import numpy as np

#Input:
#    It takes the starting point and final destination
#Outputs:
#    The trajectory to follow to reach the goal
occupancy_grid = np.zeros((80, 120))
Map = update_map_2.UpdateMap()
occupancy_grid = Map.initialize_map(occupancy_grid)
print(occupancy_grid)

starting_point = "A"
goal_point = "C"
C_Clockwise = False

if C_Clockwise:
    A_point = [Map.A[0][0] + (Map.A[2][0] - Map.A[0][0])/4, (Map.A[1][1] - Map.A[0][1])/2]
    B_point = [Map.B[0][0] + (Map.B[2][0] - Map.B[0][0])/4, Map.B[0][1] + 3*(Map.B[1][1] - Map.B[0][1])/4]
    C_point = [Map.C[0][0] + 3*(Map.C[2][0] - Map.C[0][0])/4, Map.C[0][1] + 3*(Map.C[1][1] - Map.C[0][1])/4]
    D_point = [Map.D[0][0] + 3*(Map.D[2][0] - Map.D[0][0])/4, (Map.D[1][1] - Map.D[0][1])/2]
    MISSIONS = {"D":D_point, "C":C_point, "B":B_point, "A": A_point}
else:
    A_point = [Map.A[0][0] + 3*(Map.A[2][0] - Map.A[0][0])/4, (Map.A[1][1] - Map.A[0][1])/2]
    B_point = [Map.B[0][0] + 3*(Map.B[2][0] - Map.B[0][0])/4, Map.B[0][1] + (Map.B[1][1] - Map.B[0][1])/4]
    C_point = [Map.C[0][0] + (Map.C[2][0] - Map.C[0][0])/4, Map.C[0][1] + (Map.C[1][1] - Map.C[0][1])/4]
    D_point = [Map.D[0][0] + (Map.D[2][0] - Map.D[0][0])/4, (Map.D[1][1] - Map.D[0][1])/2]
    MISSIONS = {"A": A_point, "B":B_point, "C":C_point, "D":D_point}


trajectory = [i for i in list(MISSIONS.values())[list(MISSIONS.keys()).index(starting_point):list(MISSIONS.keys()).index(goal_point)+1]]
print(trajectory)

## HERE IS THE LOGIC OF HOW WE SHOULD USE THE PLANNING, INTEGRATED INTO THE NAVIGATION
#for i in trajectory:
#    if (drone_location[0]-i[0])**2 + (drone_location[1]-i[1])**2 < 1:
#        turn 90 degrees
#    else if (drone_location[0]-i[0])**2 + (drone_location[1]-i[1])**2 < 3:
#        check the front range sensor
#    else:
#        go forward
#Land




#generate_checkpoints = {i for i in list(MISSIONS.keys())[list(MISSIONS.keys()).index(starting_point):list(MISSIONS.keys()).index(goal_point)+1]}

