# a* algorithm
# author : 7th group in project class
# 5 motions ( up, down, left, right, stop )
# input : 2-D map text file
# output : a* optimal path

import numpy as np
import math
import heapq
import os
import socket
import pickle

class Astaralgorithm:
    def __init__(self, map):
        # initialize the class
        # map = self.map
        self.map = map

        # function compute_heuristic
        # input : map, goal
        # output : heuristic value variable h_values
        # function : calculate all the heuristic values to the goal in the map
        #            ( using Manhattan distance )

    def compute_heuristic(self, map, goal):
        h_values = dict()
        for x in range(len(self.map)):
            for y in range(len(self.map[0])):
                if (self.map[x][y] == False):
                    # Manhattan distance
                    h_values[(x,y)] = abs(goal[0] - x) + abs(goal[1] - y)
        return h_values

    def astarplanning(self, start_loc, goal_loc):
        open_list = [] # openlist
        closed_list = dict()  # closed list set
        # start location 1. insert the start location in the priority queue
        h_values = self.compute_heuristic(self.map, goal_loc) # calculate all the heuristic value in the map to the goal
        h_value =  h_values[start_loc]                         # h_value is the heuristic value from start to the goal
        root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None}
        self.push_node(open_list, root)

        # 2.
        closed_list[(root['loc'])] = root

        # 3.
        while len(open_list) > 0 :
            current_node = self.pop_node(open_list)

            if( current_node ['loc'] == goal_loc) :
                print("find goal")
                return self.get_path(current_node)

            for dir in range(5):
                child_loc = self.next(current_node['loc'], dir)

                # when move direction out of the map
                if (child_loc[0] < 0 or child_loc[0] > len(self.map)) or (child_loc[1] > len(self.map) or child_loc[1] < 0):
                    continue

                # when meet obstacle
                if self.map[child_loc[0]][child_loc[1]]:
                    continue
                # g_val is always + 1 because always cost is 1
                child = {'loc': child_loc, 'g_val' : current_node['g_val']+1, 'h_val' : h_values[child_loc], 'parent' : current_node}

                # if child state is already in closed list, compare the f-value(g+h) and store the better one
                # the meaning it's already in closed list, there is no that state in open list, so we need to add it in open list
                if (child['loc'] in closed_list):
                    existing_node = closed_list[child['loc']]
                    if self.compare_fvalue(child, existing_node):
                        closed_list[(child['loc'])] = child
                        self.push_node(open_list, child)
                else :
                    closed_list[child['loc']] = child
                    self.push_node(open_list, child)

        return None # fail to path finding



    # push_node function
    # input : open_list, node
    # output: none
    # function : insert the 1. f = g + h value
    #                       2. h value
    #                       3. location
    #                       4. node
    #            values in the open_list
    #

    def push_node(self, open_list, node):
        heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

    # pop_node function
    # input : open_list
    # output : the least F ( heuristic + start to present node cost ) value node

    def pop_node(self, open_list):
        _, _, _, curnode = heapq.heappop(open_list) # open list에서 node 꺼냄
        return curnode

    # get_path function
    # input : goal_node
    # output: path from the start to goal
    # function : 1. track the goal to start parent node ( goal to start )
    #            2. reverse the path node ( start to goal )
    #
    #            can find the path from the start to goal
    #
    def get_path(self, goal_node):
        path = []
        current_node = goal_node
        while current_node is not None :
            path.append(current_node['loc'])
            current_node = current_node['parent']
        path.reverse()
        return path

    # next function
    # input : current_loc, direction 0 ~ 4
    # output: next move location
    #
    def next(self, current_loc, dir):
        move_directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
        return current_loc[0] + move_directions[dir][0], current_loc[1] + move_directions[dir][1]

    # compare_fvalue function
    # input : node1, node2
    # output : if node1 f value < node2 f value return true
    #          else return false
    def compare_fvalue(self,node1, node2):
        return node1['g_val'] + node1['h_val'] < node2['g_val'] + node2['h_val']



def import_mapf_instance():
    f = open("./map.txt", 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    # #rows lines with the map
    map = []
    obstacles = []

    for r in range(rows):
        line = f.readline()
        map.append([])
        temp_col = 0
        for cell in line:
            if cell == '@':
                map[-1].append(True)
                obstacles.append((r,temp_col))
                temp_col += 1
            elif cell == '.':
                map[-1].append(False)
                temp_col += 1
    f.close()
    return map, obstacles

def print_mapf_instance(my_map, start, goal):
    print('Start location')
    print_location(my_map, start)
    print('Goal location')
    print_location(my_map, goal)

def print_location(my_map, location):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    starts_map[location[0]][location[1]] = 0
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)

def main():
    # the Host IP address
    HOST = "172.20.10.8"

    # TCP Port #
    PORT = 9999
    # make socket
    # address family : IPv4, socket type : TCP
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # with Host and Port # connect the server
    client_socket.connect((HOST, PORT))

    client_socket.sendall('Start location ? '.encode())

    # message receive ( start location )
    data = client_socket.recv(1024)
    print('Received', repr(data.decode()))

    # start location parsing
    start = tuple(int(x) for x in data.split())
    print("start loc : ", start)

    # input the goal location
    # map instance
    map, obstacles = import_mapf_instance()
    goals = input("\nInput the goal :\n")
    goals_split = goals.split()
    goals_list = [(int(goals_split[0]), int(goals_split[1]))]
    goal = goals_list[0]
    while (goal in obstacles) :
        goals = input("\nObstacle! Retry input the goal : \n")
        goals_split = goals.split()
        goals_list = [(int(goals_split[0]), int(goals_split[1]))]
        goal = goals_list[0]
    print_mapf_instance(map, start, goal)

    # calculate route with A* algorithm
    a_star = Astaralgorithm(map)
    path = a_star.astarplanning(start, goal)

    path_path = "C:/Users/ilena/Desktop/path.txt"

    with open(path_path, "w+") as lf :
        for r in path :
            lf.write(str(r))
            lf.write("\n")
    lf.close()

    copy = '0'
    f = open(path_path, 'r')
    s = f.read()
    f.close()

    # send to the server the route
    print("path : \n", s)
    copy = s
    client_socket.sendall(s.encode())

    while True :
        # get the location from the raspberry
        data = client_socket.recv(1024)
        print('Received', repr(data.decode()))

        # when data is '1' the raspberry reach the dest
        if (data.decode() == '1') :
            break

        start = tuple(int(x) for x in data.split())

        # calculate path and write the path on the "txt" file
        path = a_star.astarplanning(start, goal)
        path_path = "C:/Users/ilena/Desktop/path.txt"
        with open(path_path, "w+") as lf :
            for r in path :
                lf.write(str(r))
                lf.write("\n")
        lf.close()

        copy = '0'
        f = open(path_path, 'r')
        s = f.read()
        f.close()

        # send to the server the route
        print("Path : \n", s)
        copy = s
        client_socket.sendall(s.encode())
    # close the socket
    client_socket.close()

if __name__ == '__main__':
    main()
