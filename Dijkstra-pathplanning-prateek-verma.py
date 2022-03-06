"""
ENPM661 - Project 2

Implementation of Dijkstra algorithm for a point robot.

Author: Prateek Verma
"""
#!/usr/bin/env python
import numpy as np
import math
import cv2
from Obstacle import *

x_limit = 400
y_limit = 250

# 
class Dijkstra_PathPlan:
    def __init__(self, start_state, goal_state):
        self.n_i = 1 # Node Index
        # Format Node: (cost2come value, node index, parent index, start state)
        self.start_state = start_state # Start node 
        self.goal_state = goal_state # Goal (x,y)
        self.goal_cost = None
        self.open = {self.start_state : (0, self.n_i, 0)}
        self.map = np.zeros([y_limit+1, x_limit+1, 3], dtype=np.uint8)
        self.map.fill(50)
        
        self.closed = None
        self.path = np.array([[]], dtype=object)
        self.exit = False
        
    # Create the obstacle map -------------------------------------------------------------------------------------------------
    def Draw_Obstacle_Map(self):
        # Concave Quad
        cv2.fillPoly(self.map, [new_con_quad], (86,86,255))
        cv2.fillPoly(self.map, [con_quad], (0,0,191))

        self.map = cv2.circle(self.map, (circle_offset_x, 250 - circle_offset_y), circle_radius, (86,86,255), thickness=-1)
        self.map = cv2.circle(self.map, (circle_offset_x, 250 - circle_offset_y), int(circle_diameter/2), (0,0,191), thickness=-1)

        cv2.fillPoly(self.map, [new_hex], (86,86,255))
        cv2.fillPoly(self.map, [arr_hex], (0,0,191))
        
        # cv2.namedWindow("map", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("map", 2000, 1250)
        # cv2.imshow("map", self.map)
        # cv2.waitKey()
        # cv2.destroyAllWindows()

        frameSize = (400, 250)
        fourcc = cv2.VideoWriter_fourcc('m','p','4','v')
        out  = cv2.VideoWriter('dijkstra_out.mp4', fourcc, 250, frameSize)
        out.write(self.map)
        for i in range(len(self.closed)):
            x = self.closed[i][3][0]
            y = y_limit - self.closed[i][3][1]
            self.map[y][x] = [191,191,0]
            out.write(self.map)
        for i in range(len(self.path)):
            x = self.path[i][0]
            y = y_limit - self.path[i][1]
            self.map[y][x] = [0,255,255]
            out.write(self.map)
        for i in range(1250):
            out.write(self.map)
        out.release()
        print('Cost To Come for Goal: ' + str(self.goal_cost))

    def isObstacle(self,X):
        if (X[0] > 394 or X[0] < 6 or X[1] < 6 or X[1] > 244):
            return True
        
        IsInCircle = ( (X[0] - circle_offset_x)**2 + (X[1] - circle_offset_y)**2 <= (circle_radius) ** 2)

        IsInHex = (X[1] >= m_hl3*X[0] + c_hl3_dash and X[1] >= m_hl4*X[0] + c_hl4_dash and X[0] >= 160 and X[0] <= 240 and X[1] <= m_hl1*X[0] + c_hl1_dash and X[1] <= m_hl6*X[0] + c_hl6_dash) 

        # IsInPolly = (X[1] <= m_l1*X[0] + c_l1_dash and X[1] >= m_l2*X[0] + c_l2_dash and X[1] <= m_l3*X[0] + c_l3_dash and X[1] >= m_l4*X[0] + c_l4_dash)
        IsInPolly = (X[1] <= m_l1*X[0] + c_l1_dash and X[1] >= m_l2*X[0] + c_l2_dash and X[1] >= m_divide*X[0] + c_divide) or (X[1] <= m_l3*X[0] + c_l3_dash and X[1] >= m_l4*X[0] + c_l4_dash and X[1] <= m_divide*X[0] + c_divide)

        return(IsInCircle or IsInHex or IsInPolly)


    # Move actions ----------------------------------------------------------------------------------------------------------------------------------------------------------
    def ActionMoveLeft(self, current_node_key):
        X = np.copy(list(current_node_key))
        if X[0] != 0:
            X[0] -= 1
            success = True
            return success, tuple(X)
        else:
            success = False
            return success, self.start_state

    def ActionMoveUp(self, current_node_key):
        X = np.copy(list(current_node_key))
        if X[1] != y_limit:
            X[1] += 1
            success = True
            return success, tuple(X)
        else:
            success = False
            return success, self.start_state

    def ActionMoveRight(self, current_node_key):
        X = np.copy(list(current_node_key))
        if X[0] != x_limit:
            X[0] += 1
            success = True
            return success, tuple(X)
        else:
            success = False
            return success, self.start_state

    def ActionMoveDown(self, current_node_key):
        X = np.copy(list(current_node_key))
        if X[1] != 0:
            X[1] -= 1
            success = True
            return success, tuple(X)
        else:
            success = False
            return success, self.start_state

    def ActionMoveLeftUp(self, current_node_key):
        X = np.copy(list(current_node_key))
        if X[0] != 0 and X[1] != y_limit:
            X[0] -= 1
            X[1] += 1
            success = True
            return success, tuple(X)
        else:
            success = False
            return success, self.start_state

    def ActionMoveRightUp(self, current_node_key):
        X = np.copy(list(current_node_key))
        if X[0] != x_limit and X[1] != y_limit:
            X[0] += 1
            X[1] += 1
            success = True
            return success, tuple(X)
        else:
            success = False
            return success, self.start_state

    def ActionMoveRightDown(self, current_node_key):
        X = np.copy(list(current_node_key))
        if X[0] != x_limit and X[1] != 0:
            X[0] += 1
            X[1] -= 1
            success = True
            return success, tuple(X)
        else:
            success = False
            return success, self.start_state

    def ActionMoveLeftDown(self, current_node_key):
        X = np.copy(list(current_node_key))
        if X[0] != 0 and X[1] != 0:
            X[0] -= 1
            X[1] -= 1
            success = True
            return success, tuple(X)
        else:
            success = False
            return success, self.start_state

    # Backtrack -----------------------------------------------------------------------------------------------------
    def backtrack(self,g):
        c = np.copy(g)
        c_i = g[1]
        indices = self.closed[:, 1]
        crd_list = self.closed[:, 3]
        while c_i != 0:
            self.path = np.append(self.path, crd_list[np.where(indices == c_i)[0]])
            c_i = c[2]
            if c_i == 0:
                break
            c = self.closed[np.where(indices == c_i)[0][0], :]
            
        self.path = np.flip(self.path, axis=0)

    # Path Planning Algorithm ----------------------------------------------------------------------------------------
    def dijkstra(self):
        j = 0 # iteration count
        while (self.open) and (not self.exit):
            # Popping from dictionary like a minimum priority queue
            n_k =  min(self.open.items(), key=lambda x: x[1][0])[0]
            n_v = self.open[n_k]
            del self.open[n_k]

            if j==0:
                # Start node
                self.closed = np.array([[n_v[0], n_v[1], n_v[2], n_k]], dtype=object) 
            else:
                self.closed = np.append(self.closed, [np.array([n_v[0], n_v[1], n_v[2], n_k], dtype=object)], axis=0)
            
            # Once goal is found.. backtrack and exit the loop.
            if n_k == self.goal_state:
                self.goal_cost = n_v[0]
                self.exit = True
                print('Goal Reached!!')
                break
            
            L_exists, L_Node = self.ActionMoveLeft(n_k)
            U_exists, U_Node = self.ActionMoveUp(n_k)
            R_exists, R_Node = self.ActionMoveRight(n_k)
            D_exists, D_Node = self.ActionMoveDown(n_k)
            LU_exists, LU_Node = self.ActionMoveLeftUp(n_k)
            RU_exists, RU_Node = self.ActionMoveRightUp(n_k)
            RD_exists, RD_Node = self.ActionMoveRightDown(n_k)
            LD_exists, LD_Node = self.ActionMoveLeftDown(n_k)

            # Check for conditions for movement
            if L_exists and (not self.isObstacle(L_Node)):
                in_closed = [n == L_Node for n in self.closed[:,3]]
                if np.count_nonzero(in_closed) ==0:
                    c2c = n_v[0] + 1
                    p_i = n_v[1]
                    if not (L_Node in self.open.keys()):
                        self.n_i += 1
                        self.open[L_Node] = (c2c, self.n_i, p_i)
                    else:
                        if(self.open[L_Node][0] > c2c):
                            self.open[L_Node] = (c2c, self.open[L_Node][1], p_i)
                        
            if U_exists and (not self.isObstacle(U_Node)):
                in_closed = [n == U_Node for n in self.closed[:,3]]
                if np.count_nonzero(in_closed) == 0:
                    c2c = n_v[0] + 1
                    p_i = n_v[1]
                    if not (U_Node in self.open.keys()):
                        self.n_i += 1
                        self.open[U_Node] = (c2c, self.n_i, p_i)
                    else:
                        if(self.open[U_Node][0] > c2c):
                            self.open[U_Node] = (c2c, self.open[U_Node][1], p_i)
            if R_exists and (not self.isObstacle(R_Node)):
                in_closed = [n == R_Node for n in self.closed[:,3]]
                if np.count_nonzero(in_closed) == 0:
                    c2c = n_v[0] + 1
                    p_i = n_v[1]
                    if not (R_Node in self.open.keys()):
                        self.n_i += 1
                        self.open[R_Node] = (c2c, self.n_i, p_i)
                    else:
                        if(self.open[R_Node][0] > c2c):
                            self.open[R_Node] = (c2c, self.open[R_Node][1], p_i)
            if D_exists and (not self.isObstacle(D_Node)):
                in_closed = [n == D_Node for n in self.closed[:,3]]
                if np.count_nonzero(in_closed) == 0:
                    c2c = n_v[0] + 1
                    p_i = n_v[1]
                    if not (D_Node in self.open.keys()):
                        self.n_i += 1
                        self.open[D_Node] = (c2c, self.n_i, p_i)
                    else:
                        if(self.open[D_Node][0] > c2c):
                            self.open[D_Node] = (c2c, self.open[D_Node][1], p_i)
            
            if LU_exists and (not self.isObstacle(LU_Node)):
                in_closed = [n == LU_Node for n in self.closed[:,3]]
                if np.count_nonzero(in_closed) == 0:
                    c2c = n_v[0] + 1.4
                    p_i = n_v[1]
                    if not (LU_Node in self.open.keys()):
                        self.n_i += 1
                        self.open[LU_Node] = (c2c, self.n_i, p_i)
                    else:
                        if(self.open[LU_Node][0] > c2c):
                            self.open[LU_Node] = (c2c, self.open[LU_Node][1], p_i)

            if RU_exists and (not self.isObstacle(RU_Node)):
                in_closed = [n == RU_Node for n in self.closed[:,3]]
                if np.count_nonzero(in_closed) == 0:
                    c2c = n_v[0] + 1.4
                    p_i = n_v[1]
                    if not (RU_Node in self.open.keys()):
                        self.n_i += 1
                        self.open[RU_Node] = (c2c, self.n_i, p_i)
                    else:
                        if(self.open[RU_Node][0] > c2c):
                            self.open[RU_Node] = (c2c, self.open[RU_Node][1], p_i)
            
            if RD_exists and (not self.isObstacle(RD_Node)):
                in_closed = [n == RD_Node for n in self.closed[:,3]]
                if np.count_nonzero(in_closed) == 0:
                    c2c = n_v[0] + 1.4
                    p_i = n_v[1]
                    if not (RD_Node in self.open.keys()):
                        self.n_i += 1
                        self.open[RD_Node] = (c2c, self.n_i, p_i)
                    else:
                        if(self.open[RD_Node][0] > c2c):
                            self.open[RD_Node] = (c2c, self.open[RD_Node][1], p_i)
            
            if LD_exists and (not self.isObstacle(LD_Node)):
                in_closed = [n == LD_Node for n in self.closed[:,3]]
                if np.count_nonzero(in_closed) == 0:
                    c2c = n_v[0] + 1.4
                    p_i = n_v[1]
                    if not (LD_Node in self.open.keys()):
                        self.n_i += 1
                        self.open[LD_Node] = (c2c, self.n_i, p_i)
                    else:
                        if(self.open[LD_Node][0] > c2c):
                            self.open[LD_Node] = (c2c, self.open[LD_Node][1], p_i)
            j += 1
        
        # Check for avaialble solution using backtrack
        if self.exit:
            self.backtrack(self.closed[-1, :])
            self.Draw_Obstacle_Map()
        else:
            print('Path Not Found!!!')
            if not self.open:
                print('Empty Open')


# Main function to execute the code..
if __name__ == '__main__':
    start = [0,0]
    goal = [400,250]
    
    # Take user input for Start and Goal position for the point robot.
    start[0] = int(input('Enter start x: '))
    start[1] = int(input('Enter start y: '))
    goal[0] = int(input('Enter goal x: '))
    goal[1] = int(input('Enter goal y: '))
    start = tuple(start)
    goal = tuple(goal)
    D = Dijkstra_PathPlan(start,goal)
    while(D.isObstacle(start) or D.isObstacle(goal)):
        if D.isObstacle(start):
            print('Start state is in obstacle')
        if D.isObstacle(goal):
            print('Goal state is in obstacle')
        print('Re-enter start and goal states: ')
        start = [0,0]
        goal = [400,250]
        start[0] = int(input('Enter start x: '))
        start[1] = int(input('Enter start y: '))
        goal[0] = int(input('Enter goal x: '))
        goal[1] = int(input('Enter goal y: '))
    start = tuple(start)
    goal = tuple(goal)
    D = Dijkstra_PathPlan(start,goal)
    # D.Draw_Obstacle_Map()
    D.dijkstra()
