import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import more_itertools as mit


n_data_points = 4
x_map = [0, 0, 140, 140, 163, 163, 428, 428, 450, 450, 715, 715, 738, 738, 1002, 1002, 1025, 1025, 1162, 1162, 962, 962, 200, 200, 0]
y_map = [0,	770, 770, 757, 757,	770, 770, 757, 757,	770, 770, 757, 757, 770, 770, 757, 757, 770, 770, 0, 0, 570, 570, 0, 0]
map_coords = [(x_value, y_map[i]) for i, x_value in enumerate(x_map)]
# Maybe I have to replace 200 with 140? The 630 is because it's 140 less than 770
A = [(0,0), (0, 200), (200, 0), (200, 200)]
B = [(0, 630),(0, 770), (200, 630), (200, 770)]
C = [(962, 630),(962, 770), (1162, 630), (1162, 770)]
D = [(962, 0),(962, 140), (1162, 0), (1162, 140)]
sectors = [A, B, C, D]
grid_size = 10
print(map_coords)


def coord2cell(coord):
    return round(coord/grid_size)

if __name__ == '__main__':
    x_size = 1200
    y_size = 800
    occupancy_grid = np.zeros((coord2cell(y_size), coord2cell(x_size)))

    ## Create horizontal lines
    for j in range(coord2cell(y_size)):
        matching_couples = [item for item in map_coords if coord2cell(item[1]) == j]
        matching_couples.sort()
        matching_couples = list(dict.fromkeys(matching_couples))
        # print(matching_couples)
        temp = 0
        for i in range(coord2cell(x_size)):
            # print(i)
            if len(matching_couples)>1:
                count = coord2cell(matching_couples[1][0]) - coord2cell(matching_couples[0][0])
                # print(matching_couples)
                if i>=coord2cell(matching_couples[0][0]) and i<=coord2cell(matching_couples[1][0]):
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
    for j in range(coord2cell(y_size)):
        for i in range(coord2cell(x_size)):
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
    print(max(i[0] for i in A))
    color_section = 20
    for j in range(coord2cell(y_size)):
        for i in range(coord2cell(x_size)):
            for sector in sectors:
                color_section += color_section
                max_x_sector = coord2cell(max(i[0] for i in sector))
                min_x_sector = coord2cell(min(i[0] for i in sector))
                max_y_sector = coord2cell(max(i[1] for i in sector))
                min_y_sector = coord2cell(min(i[1] for i in sector))
                if i>=min_x_sector and i<=max_x_sector and j>=min_y_sector and j<=max_y_sector and occupancy_grid[j, i]==100:
                    occupancy_grid[j, i] = 50

    ## Colour the road channels

    #if in sectionAB or sectionCD:
    #   direction = ydir
    #   if RHT:
    #       for j in range(coord2cell(y_size)):
    #           for i in range(coord2cell(x_size)):
    #               if not(i>=min_x_channel and i<=max_x_channel) and occupancy_grid[j, i]==100:
    #                   occupancy_grid[j, i]=50
    #else direction x

    #occupancy_grid=np.flip(occupancy_grid,0)
    #print(occupancy_grid)
    matplotlib.rc('xtick', labelsize=5)
    matplotlib.rc('ytick', labelsize=5)
    plt.figure()
    plt.imshow(occupancy_grid, cmap="gray")
    plt.title("Drone map")
    plt.gca().invert_yaxis()
    plt.show()
