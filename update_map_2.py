import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import more_itertools as mit
import time

class UpdateMap:

    #def __init__(self, link_uri):
    def __init__(self):

        # Initializations
        x_map = [0, 0, 140, 140, 163, 163, 428, 428, 450, 450, 715, 715, 738, 738, 1002, 1002, 1025, 1025, 1162, 1162, 962, 962, 200, 200, 0]
        y_map = [0,	770, 770, 757, 757,	770, 770, 757, 757,	770, 770, 757, 757, 770, 770, 757, 757, 770, 770, 0, 0, 570, 570, 0, 0]
        self.map_coords = [(x_value, y_map[i]) for i, x_value in enumerate(x_map)]
        # Maybe I have to replace 200 with 140? The 630 is because it's 140 less than 770
        self.A = [(0,0), (0, 200), (200, 0), (200, 200)]
        self.B = [(0, 570),(0, 770), (200, 570), (200, 770)]
        self.C = [(962, 570),(962, 770), (1162, 570), (1162, 770)]
        self.D = [(962, 0),(962, 200), (1162, 0), (1162, 200)]
        self.sectors = [self.A, self.B, self.C, self.D]
        self.grid_size = 10
        print(self.map_coords)
        self.C_Clockwise = False




    def plot_grid(self, occupancy_grid_upd):
        # Set plot to animated
        plt.ion() 
        # Define a figure box of certain size
        matplotlib.rc('xtick', labelsize=5)
        matplotlib.rc('ytick', labelsize=5)
        plt.figure(figsize=(10.0, 11.5))
        plt.title("Drone map")
        #occupancy_grid_upd = self.initialize_map(occupancy_grid)
        self.Occ_Map = plt.imshow(occupancy_grid_upd, cmap="gray")
        plt.gca().invert_yaxis()
        plt.grid(linestyle = '--', linewidth = 0.2)

        while True:
            self.Update_map(occupancy_grid_upd, Occ_Map)
            plt.pause(0.01)
        
    def coord2cell(self, coord):
        return round(coord/self.grid_size)

    def initialize_map(self, occupancy_grid):
        self.x_size = 1200
        self.y_size = 800
        occupancy_grid = np.zeros((self.coord2cell(self.y_size), self.coord2cell(self.x_size)))
        ## Create horizontal lines
        for j in range(self.coord2cell(self.y_size)):
            matching_couples = [item for item in self.map_coords if self.coord2cell(item[1]) == j]
            matching_couples.sort()
            matching_couples = list(dict.fromkeys(matching_couples))
            # print(matching_couples)
            temp = 0
            for i in range(self.coord2cell(self.x_size)):
                # print(i)
                if len(matching_couples)>1:
                    count = self.coord2cell(matching_couples[1][0]) - self.coord2cell(matching_couples[0][0])
                    # print(matching_couples)
                    if i>=self.coord2cell(matching_couples[0][0]) and i<=self.coord2cell(matching_couples[1][0]):
                        occupancy_grid[j, i]=100
                        temp += 1
                        # print(temp)
                        if temp == count:
                            matching_couples.remove(matching_couples[0])
                            matching_couples.remove(matching_couples[0])
                            temp = 0
                    else:
                        continue

        ## Fill the space between the horizontal lines
        #print([i for i,x in enumerate(occupancy_grid[:,0]) if x==100])
        for j in range(self.coord2cell(self.y_size)):
            for i in range(self.coord2cell(self.x_size)):
                occupied_spaces = [i for i,x in enumerate(occupancy_grid[:,i]) if x==100]
                # print(occupied_spaces)
                occupied_spaces = [list(group) for group in mit.consecutive_groups(occupied_spaces)]
                # print(occupied_spaces)
                if len(occupied_spaces)>1:
                    occupied_spaces = [max(occupied_spaces[0]), min(occupied_spaces[1])]
                    # print(occupied_spaces)
                    if j>=occupied_spaces[0] and j<=occupied_spaces[1]:
                        occupancy_grid[j, i]=100

        ## Colour the sectors
        print(max(i[0] for i in self.A))
        color_section = 20

        for j in range(self.coord2cell(self.y_size)):
            for i in range(self.coord2cell(self.x_size)):
                for sector in self.sectors:
                    color_section += color_section
                    max_x_sector = self.coord2cell(max(k[0] for k in sector))
                    min_x_sector = self.coord2cell(min(k[0] for k in sector))
                    max_y_sector = self.coord2cell(max(k[1] for k in sector))
                    min_y_sector = self.coord2cell(min(k[1] for k in sector))
                    if i>=min_x_sector and i<max_x_sector and j>=min_y_sector and j<=max_y_sector and occupancy_grid[j, i]==100:
                        occupancy_grid[j, i] = 50

        ## Colour the road channels
        for j in range(self.coord2cell(self.y_size)):
            for i in range(self.coord2cell(self.x_size)):
                # Section AB
                if i>=self.coord2cell(self.A[0][0]) and i<=self.coord2cell(self.A[2][0]) and j>=self.coord2cell(self.A[1][1]) and j<=self.coord2cell(self.B[0][1]) and occupancy_grid[j, i]==100:
                    occupancy_grid[j, i] = 10
                    if self.C_Clockwise:
                        if i>=self.coord2cell(self.A[2][0])/2 and i<=self.coord2cell(self.A[2][0]) and j>=self.coord2cell(self.A[1][1]) and j<=self.coord2cell(self.B[0][1]) and occupancy_grid[j, i]==10:
                            occupancy_grid[j, i] = 0
                    else:
                        if i>=self.coord2cell(self.A[0][0]) and i<=self.coord2cell(self.A[2][0])/2 and j>=self.coord2cell(self.A[1][1]) and j<=self.coord2cell(self.B[0][1]) and occupancy_grid[j, i]==10:
                            occupancy_grid[j, i] = 0
                # Section BC
                if i>=self.coord2cell(self.B[2][0]) and i<=self.coord2cell(self.C[0][0]) and j>=self.coord2cell(self.B[2][1]) and j<=self.coord2cell(self.B[3][1]) and occupancy_grid[j, i]==100:
                    occupancy_grid[j, i] = 20
                    if self.C_Clockwise:
                        if i>=self.coord2cell(self.B[2][0]) and i<=self.coord2cell(self.C[0][0]) and j>=self.coord2cell(self.B[2][1]) and j<=(self.coord2cell(self.B[2][1])+(self.coord2cell(self.B[3][1])-self.coord2cell(self.B[2][1]))/2) and occupancy_grid[j, i]==20:
                            occupancy_grid[j, i] = 0
                    else:
                        if i>=self.coord2cell(self.B[2][0]) and i<=self.coord2cell(self.C[0][0]) and j>=(self.coord2cell(self.B[2][1])+(self.coord2cell(self.B[3][1])-self.coord2cell(self.B[2][1]))/2) and j<=self.coord2cell(self.B[3][1]) and occupancy_grid[j, i]==20:
                            occupancy_grid[j, i] = 0
                # Section CD
                if i>=self.coord2cell(self.C[0][0]) and i<=self.coord2cell(self.C[2][0]) and j>=self.coord2cell(self.D[1][1]) and j<=self.coord2cell(self.C[0][1]) and occupancy_grid[j, i]==100:
                    occupancy_grid[j, i] = 30
                    if self.C_Clockwise:
                        if i>=self.coord2cell(self.C[0][0]) and i<=(self.coord2cell(self.C[0][0])+(self.coord2cell(self.C[2][0])-self.coord2cell(self.C[0][0]))/2) and j>=self.coord2cell(self.D[1][1]) and j<=self.coord2cell(self.C[0][1]) and occupancy_grid[j, i]==30:
                            occupancy_grid[j, i] = 0
                    else:
                        if i>=(self.coord2cell(self.C[0][0])+(self.coord2cell(self.C[2][0])-self.coord2cell(self.C[0][0]))/2) and i<=self.coord2cell(self.C[2][0]) and j>=self.coord2cell(self.D[1][1]) and j<=self.coord2cell(self.C[0][1]) and occupancy_grid[j, i]==30:
                            occupancy_grid[j, i] = 0

        return occupancy_grid


    def Update_map(self, occupancy_grid_upd, Occ_Map):
        #occupancy_grid[0, 0] = 0
        Occ_Map.set_data(occupancy_grid_upd)
        plt.draw()
        plt.pause(0.01)

