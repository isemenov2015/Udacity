import numpy as np
import random
import sys
from graph import Graph

class Robot(object):
    
    def __init__(self, maze_dim, mode = 'solver'):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        self.location = np.array([0, 0])
        self.heading = 'up'
        self.maze_dim = maze_dim
        self.connectivity_graph = Graph(maze_dim) #connectivity graph
        self.steps_counter = 0 #time steps counter
        self._orientation = {'up': (1, 0), 'down': (-1, 0), \
                            'left': (0, -1), 'right': (0, 1)}
        self._unvisited = list() #list of a cells to visit during exploration phase with 'benchmark' mode
        self._orientation_sequence = ('up', 'right', 'down', 'left')
        self.visited = set() #set of visited cells during exploration phase
        self._visits_counter = dict() # dictionary for counting cell visits during exploration
        self._reachable = set() #set of cells that are reachable but still not visited. Used in 'solver' mode
        self.max_sensor_view_range = 3 #max sensor sight range in cells
        self.max_move_range = 3 #max cells robot can move forward in one time-step
        self.mode = mode  #exploration mode: 'benchmark' for slow full exploration or 'solver' for the fast one
        self.path = list() #real path of a robot during all runs
        self._local_path = list() #helper list for storing paths towards next unvisited cell
        self.run = 1 #Number of run. 1 - exploration phase, 2 - first step of a final phase, 3 - other steps of final phase
        self.stop_run = False # inner flag that represents current run needs to stop
        self._print_visits_counter = False # inner switcher. It True, outputs self._visits_counter to a text file
        
        # Exploration phase tuning constants
        self._SHORT_PATH_COEFF = 0.1 # used for calculation of immediate stop criteria during exploration phase
        self._TOTAL_STEPS = 1000 # No of total steps allowed for maze travelling
        self._PROB_REDUCE = 0.12 # probability reduce coefficient for random stop when probably suboptimal path found
        self._SECOND_VISIT_PENALTY = 100 # penalty coefficient for repeatitive visit to the same cell
        self._CENTER_MOVEMENT_BONUS = 5 # weight bonus coefficient for movement towards the center
        
        random.seed(100)

    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        
        #print 'Step No', self.get_moves(), 'Robot in a cell No', self.cell()
        self._reachable.discard(self.cell())
        if self.run == 2:
            #print "Final phase"
            self.location = [0, 0]
            self.heading = 'up'
            self._local_path = list()
            self.visited = set()
            print 'Exploration path length:', len(self.get_path())
            if self._print_visits_counter:
                self._write_visits_counter()
            path_len = len(self._get_shortest_path(self.cell(), self._target_coord()[0]))
            if self.mode == 'benchmark':
                self._next_cell_benchmark()
                #print self._local_path
                #print self._truncate_path(self._local_path)
                #print self.get_path()
                path_len = len(self._truncate_path(self._local_path)) + 2
                self._local_path = list()
            shortest_path_len = path_len - 1
            print 'Shortest path length:', shortest_path_len
            if self.mode == 'benchmark':
                print 'Best possible score:', shortest_path_len * 1.0/30 + shortest_path_len
                print 'Slow deterministic exploration score:', len(self.get_path()) * 1.0 / 30 + shortest_path_len
            self.run = 3
        if self.mode == 'benchmark':
            rotation, movement = self._benchmark(sensors)
        else:
            rotation, movement = self._strategy(sensors)
        if not (rotation == 0 and movement == 0):
            self.steps_counter += 1
        self.path.append((self.cell(), self.heading))
        #if self.cell() in self._target_coord():
        #    print 'Center reached!'
        #    print 'Current cell:', self.cell()
        return rotation, movement
        
    def _get_shortest_path(self, source, target):
        """
        Returns a list with shortest path from current cell to the center
        based on a connectivity graph
        """
        path = self.connectivity_graph.search_path(source, target)
        if not source in self._target_coord():
            while len(path) > 1 and (path[-2] in self._target_coord()):
                path = path[:-1]
        return path
        
    def _strategy(self, sensors):
        """
        Returns next move in a solver strategy mode
        """
        
        if self.run > 1:
            if self.stop_run:
                self.stop_run = False
                return 'Reset', 'Reset'
            #print self._next_cell_benchmark()
            #input('Press Enter')
            path = self._get_shortest_path(self.cell(), self._target_coord()[0])
            if not path:
                # intended to never get there
                print 'Internal error encountered. Please, run again'
                print 'Robot path:', self.path
                sys.exit()
            next_cell = path[1] if self.cell() != 0 else path[0]
            #print 'Path found:', path, 'Location:', self.location, 'Heading:', self.heading
            #input('Press Enter')
            rotation, movement = self._find_move_solver(next_cell)
            #print 'Final run. Step:', self.get_moves(), 'Cell:', self.cell(), 'Heading:', self.heading, 'Rotation:', rotation, 'Movement:', movement
            #print 'Path:', path
            self.location, self.heading = self._update_position(rotation, movement)
            return rotation, movement
        self._update_graph(sensors)
        rotation, movement = self._next_cell_solver()
        self.location, self.heading = self._update_position(rotation, movement)
        self.stop_run = self._stop_condition()
        if self.run == 1 and self.stop_run:
            self.run = 2
            #print 'Run changed to 2, Cell:', self.cell(), 'Movement:', movement, 'Rotation:', rotation
            #input('Press Enter')
        return rotation, movement
        
    def _stop_condition(self):
        """
        Returns True if random condition for exploration phase termination satisfied
        """
        
        stop = False
        start = 0
        finish = self._target_coord()[0]
        path = self.connectivity_graph.search_path(start, finish)
        if (path != None) and (len(path) <= (self.maze_dim ** 2) * self._SHORT_PATH_COEFF):
            print 'Stopped exploration on finding satisfactory short path'
            stop = True
        elif (not stop) and (path != None): # random stop when maybe suboptimal path is found
            if random.random() < self._PROB_REDUCE * self.get_moves() / self._TOTAL_STEPS:
                print 'Stopped exploration on random condition for probably suboptimal path'
                #print 'Path:', path, 'Visited:', self.visited
                stop = True
        return stop
        
    def _next_cell_solver(self):
        """
        Returns rotation and movement for the robot to get to the next cell in
        the exploration phase for the 'solver' mode
        """
        
        neighbors_set = set(self.connectivity_graph.get_neighbors(self.cell()))
        #print 'Current cell:', self.cell(), 'Neighbors set:', neighbors_set
        possible_moves = list(neighbors_set)
        moves = np.array(possible_moves)
        weights = list()
        for cell in possible_moves:
            layer_difference = self.connectivity_graph.get_layer(self.cell()) - self.connectivity_graph.get_layer(cell)
            layer_difference *= self._CENTER_MOVEMENT_BONUS #enhance center targeting
            weights.append(layer_difference + 0.0001)
        #print 'moves:', moves, 'weights:', weights
        #print self.connectivity_graph
        for i in range(len(weights)):
            if possible_moves[i] in self.visited:
                #print self._visits_counter
                weights[i] -= self._SECOND_VISIT_PENALTY * self._visits_counter[possible_moves[i]]
            if (possible_moves[i] in self._target_coord()) and (not possible_moves[i] in self.visited):
                weights[i] = sys.maxint # must visit central cell if it is possible and it has not been visited yet
        #update Unvisited nodes set
        self._reachable = self._reachable.union(neighbors_set.difference(self.visited))
        for target_cell in self._reachable:
            for i in range(len(weights)):
                path = self.connectivity_graph.search_path(target_cell, possible_moves[i])
                if path != None and possible_moves[i] in path:
                    layer_target = self.connectivity_graph.get_layer(target_cell)
                    layer_source = self.connectivity_graph.get_layer(self.cell())
                    weights[i] += 1.0 * (layer_source - layer_target) / max(len(path), 1)
                    #print 'layer_target:', layer_target, 'layer_source:', layer_source, 'target:', target_cell, 'source:', possible_moves[i], 'i:', i
        weights = np.array(weights)
        #print 'Unnormalized weights:', weights, 'Unvisited set:', self._reachable, 'Visited set:', self.visited
        #input('Press Enter')
        if np.max(weights) <= 0:
            weights += abs(min(weights))
        else:
            pos = np.argwhere(weights <= 0)
            moves = np.delete(moves, pos)
            weights = weights[weights > 0]
        weights /= np.sum(weights)
        #print 'moves:', moves, 'weights:', weights
        target_cell = np.random.choice(moves, 1, p=weights)
        #print 'Cell selected: ', target_cell
        #print 'Sequence:', self._find_move_solver(target_cell)
        #print 'Path so far:', self.get_path()
        return self._find_move_solver(target_cell)
        
    def _find_move_solver(self, target):
        """
        Returns rotation and movement from current to neighboring target cell
        in a solver mode
        """
        loc = np.array(self.location)
        head = self.heading
        rot = 0
        for step in range(1, 4):
            new_loc, new_head = self._update_position(rot, step)
            if self.cell(new_loc) == target:
                self.location = loc
                self.heading = head
                return rot, step
        for step in range(-3, 0):
            new_loc, new_head = self._update_position(rot, step)
            if self.cell(new_loc) == target:
                self.location = loc
                self.heading = head
                return rot, step
        rot = 90
        for step in range(1, 4):
            new_loc, new_head = self._update_position(rot, step)
            if self.cell(new_loc) == target:
                self.location = loc
                self.heading = head
                return rot, step
        rot = -90
        for step in range(1, 4):
            new_loc, new_head = self._update_position(rot, step)
            if self.cell(new_loc) == target:
                self.location = loc
                self.heading = head
                return rot, step
        return 0, 0 # never get there
        
    def _truncate_path(self, path):
        """
        Shortens path for 'benchmark' mode by replacing one-cell-forward moves
        with two or three cells forward where possible
        Used during final run
        """
        #print path
        new_path = list()
        while len(path) > 0:
            move = path[-1]
            if len(new_path) > 0 and \
                 move[0] == 0 and new_path[-1][1] < self.max_move_range:
                     new_path[-1] = (new_path[-1][0], new_path[-1][1] + 1)
            else:
                new_path.append(move)
            path = path[:-1]
        #print list(reversed(new_path))
        return list(reversed(new_path))
        
    def _benchmark(self, sensors):
        """
        Returns next move in a benchmark mode
        """

        if self.run > 1:
            if self.cell() in self._target_coord():
                return 'Reset', 'Reset'
            return self._next_cell_benchmark()
        self._update_graph(sensors)
        #print 'Unvisited cells list:', self._unvisited, 'Current cell:', self.cell()
        while (len(self._unvisited) > 0) and (self._unvisited[-1] in self.visited):
            self._unvisited.pop()
        if len(self.visited) == self.maze_dim ** 2 or len(self._unvisited) == 0:
            rotation = 'Reset'
            movement = 'Reset'
            #print self.get_path()
            #print 'Visited cells:', self.visited
            self.run = 2
        else:
            rotation, movement = self._next_cell_benchmark()
            self.location, self.heading = self._update_position(rotation, movement)
        return rotation, movement
        
    def _next_cell_benchmark(self):
        """
        Returns rotation and movement for the robot to get to the next cell in
        the exploration phase for the 'benchmark' mode and for final phase
        for any modes
        """
        
        if len(self._local_path) > 0:
            step = self._local_path.pop()
            return step[0], step[1]
        else:
            if self.run == 1:
                target = self._unvisited.pop()
            else:
                target = self._target_coord()[0]
            local_path = self._get_shortest_path(self.cell(), target)
            if local_path[0] == self.cell():
                local_path = local_path[1:]
            local_path = self._convert_to_moves(local_path)
            if self.run > 1:
                local_path = self._truncate_path(local_path)
            self._local_path = local_path[:-1]
            return local_path[-1][0], local_path[-1][1]
            
    def _target_coord(self):
        """
        returns tuple with numbers of a target cells
        """
        
        v0 = (self.maze_dim - 1) * self.maze_dim / 2 - 1
        v1 = v0 + 1
        v2 = (self.maze_dim + 1) * self.maze_dim / 2 - 1
        v3 = v2 + 1
        return (v0, v1, v2, v3)
            
    def _convert_to_moves(self, path_cells):
        """
        Convert list of cell numbers to list of (Rotation, Movement) tuples
        Returns list of tuples
        """
        
        path_tuples = list()
        loc = np.array(self.location)
        head = self.heading
        for cell in path_cells:
            for rot in (90, 0, -90):
                new_loc, new_head = self._update_position(rot, 1)
                #print 'Current cell in path_cells:', cell, 'Compared with:', self.cell(new_loc), 'Heading:', new_head
                if self.cell(new_loc) == cell:
                    #print 'Match!'
                    path_tuples.append((rot, 1))
                    self.location = new_loc
                    self.heading = new_head
                    break
                if rot == -90: # round turn
                    path_tuples.append((-90, 0))
                    self.heading = new_head
                    path_tuples.append((-90, 1))
                    new_loc, new_head = self._update_position(-90, 1)
                    self.location = new_loc
                    self.heading = new_head
        self.location = loc
        self.heading = head
        return list(reversed(path_tuples))
        
    def cell(self, loc = list()):
        """
        Returns maze cell number based on a robot location. Cells enumerated
        from left to rights, from botton to top. Assumed bottom-top orientation
        that means starting cell has number 0 and located in a lower-left
        maze corner
        """
        
        if len(loc) == 0:
            loc = self.location
        return loc[0] * self.maze_dim + loc[1]
        
    def get_moves(self):
        """
        Getter for robot moves counter in a current session
        """
        
        return self.steps_counter
        
    def get_path(self):
        """
        getter for path so far
        """
        
        return self.path

    def _update_position(self, rotation, movement):
        """
        Updates robot position by given movement
        """
        rotation_index = self._orientation_sequence.index(self.heading)
        if rotation == 90:
            rot_addon = 1
        elif rotation == -90:
            rot_addon = -1
        else:
            rot_addon = 0
        if rot_addon != 0:
            new_heading = self._orientation_sequence[(rotation_index + rot_addon) % len(self._orientation_sequence)]
        else:
            new_heading = self.heading
        addon = np.array(self._orientation[new_heading])
        addon *= movement
        new_location = self.location + addon
        return new_location, new_heading
        
    def _update_graph(self, sensors):
        """
        Updates maze connectivity graph given sensors info
        """
        
        self.visited.add(self.cell())
        if self.cell() in self._visits_counter.keys():
            self._visits_counter[self.cell()] += 1
        else:
            self._visits_counter[self.cell()] = 1
        loc = np.array(self.location)
        head = self.heading
        current_cell = self.cell()
        rotation = -90
        for dir in range(len(sensors)):
            mov = 1
            vert_set = set([])
            while mov <= min(sensors[dir], self.max_sensor_view_range):
                self.location, self.heading = self._update_position(rotation, mov)
                if mov == 1 and self.mode == 'benchmark':
                    self.connectivity_graph.add_edge(current_cell, self.cell())
                    self._unvisited.append(self.cell())
                elif self.mode != 'benchmark':
                    vert_set.add(self.cell())
                    self.connectivity_graph.add_edge(current_cell, self.cell())
                for v1 in vert_set:
                    for v2 in vert_set:
                        if v1 != v2:
                            self.connectivity_graph.add_edge(v1, v2)
                self.location = loc
                self.heading = head
                mov += 1
            rotation += 90
        return
        
    def _write_visits_counter(self):
        """
        writes self._visits_counter dictionary to a text file
        for future analysis
        """
        with open('counter.txt', 'w') as file:
            for i in range(self.maze_dim ** 2):
                if i in self._visits_counter.keys():
                    file.write(str(self._visits_counter[i]))
                else:
                    file.write(str(0))
                if (i + 1) % self.maze_dim == 0:
                    file.write('\n')
                else:
                    file.write(',')
        return