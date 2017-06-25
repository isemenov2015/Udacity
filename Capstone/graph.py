# -*- coding: utf-8 -*-
"""
Created on Tue May 16 02:57:00 2017

@author: Ilya Semenov
"""

class Graph(object):
    '''
    Implements Graph data structure with basic element access/search functions
    '''
    
    def __init__(self, maze_dim):
        '''
        Creates vertices and edges sets, keeps basic data structure
        parameters
        '''
        
        self.vertice_set = dict()
        self.maze_dim = maze_dim
        for i in range(maze_dim * maze_dim):
            self.add_vertice(i)
        #Connect four central vertices with four edges
        vert1 = (self.get_dimension() - 1) * self.get_dimension() / 2 - 1
        vert2 = vert1 + 1
        vert3 = (self.get_dimension() + 1) * self.get_dimension() / 2 - 1
        vert4 = vert3 + 1
        self.add_edge(vert1, vert2)
        self.add_edge(vert1, vert3)
        self.add_edge(vert4, vert2)
        self.add_edge(vert4, vert3)
        
    def __str__(self):
        s = ''
        for v in self.vertice_set.keys():
            s += ' ' + str(v) + ': ' + str(self.vertice_set.get(v))
        return s
            
    def add_vertice(self, vertice):
        """
        Adds a vertice to a graph. 
        Has a preliminary check if a vertice already exists
        """
        
        if (vertice >= self.get_dimension()**2 or vertice < 0):
            print 'Tried to add vertice with incorrect No:', vertice
            return None
        if (not self.vertice_set.has_key(vertice)):
            self.vertice_set[vertice] = set([])
        
    def add_edge(self, vertice1, vertice2):
        """
        Adds a bidirectional edge between two vertices
        """
        
        if (vertice1 == vertice2):
            print 'Tried to add self-connected vertice:', vertice1
            return None
        self.add_vertice(vertice1)
        self.add_vertice(vertice2)
        self.vertice_set[vertice1].add(vertice2)
        self.vertice_set[vertice2].add(vertice1)
        
        #get_layer() test
        #for v in [0, 33, 120, 91, 25, 106, 66]:
        #    print 'For vertice No', v, 'Layer is:', self.get_layer(v)
        #input('Press a key to continue')
        #end of get_layer() test
        
    def has_vertice(self, vertice):
        """
        Returns True if Graph has a vertice with number given as input
        """
        
        return self.vertice_set.has_key(vertice)
        
    def has_edge(self, vertice1, vertice2):
        """
        Returns True if graph has an edge between Vertice1 and Vertice2
        """
        
        if (not self.vertice_set.has_key(vertice1)) or \
            (not (self.vertice_set.has_key(vertice2))):
                return False
        return vertice2 in self.vertice_set[vertice1]
        
    def get_dimension(self):
        """
        Getter for maze dimension
        """
        
        return self.maze_dim
        
    def get_neighbors(self, vertice):
        """
        Getter for vertice neighbors set
        """
        
        return self.vertice_set[vertice]
        
    def get_layer(self, vertice):
        """
        Returns layer number for a given vertice
        Maximum layer number defined by maze dimension
        For maze with dimension 12 maximum layer number is 5
        Four central cells of a maze belong to layer 0
        Starting cell belongs to layer with maximum number.
        For maze dimension 12:
        starting cell (number 0) belongs to layer 5
        cell with number 33 belongs to layer 3
        cell with number 120 belongs to layer 5
        cell with number 91 belongs to layer 1
        cell with number 25 belongs to layer 4
        cell with number 106 belongs to layer 4
        cell with number 66 belongs to layer 0
        """
        
        row_number = (vertice / self.get_dimension() - self.get_dimension()/2 \
                        if vertice <= self.get_dimension()**2/2 \
                        else vertice / self.get_dimension() - self.get_dimension()/2 - 1) + 1
        col_number = vertice % self.get_dimension() - self.get_dimension()/2 + 1 \
                        if vertice % self.get_dimension() < self.get_dimension()/2 \
                        else vertice % self.get_dimension() - self.get_dimension()/2
        return max(abs(row_number), abs(col_number))
        
    def search_path(self, source, target):
        """
        Returns shortest path between Source and Target as a list of
        vertices. Uses simplified Dijkstra approach since all edges have
        equal lengths
        """
        
        if not self.has_vertice(source) or not self.has_vertice(target):
            print 'Incorrect call to search_path() method. One or more vertices not found:', \
                source, target
            return None
        
        nodes_to_visit = {source}
        visited_nodes = set()
        distance_from_start = {source: 0}
        tentative_parents = {}
        while nodes_to_visit:
            current = min([(distance_from_start[node], node) \
                            for node in nodes_to_visit])[1]
            if current == target:
                break
            nodes_to_visit.discard(current)
            visited_nodes.add(current)
            edges = self.get_neighbors(current)
            #print 'Debug print. List of neighbors for vertice No', current, ':', edges
            unvisited_neighbors = edges.difference(visited_nodes)
            for neighbor in unvisited_neighbors:
                neighbor_distance = distance_from_start[current] + 1
                if neighbor_distance < distance_from_start.get(neighbor, float('inf')):
                    distance_from_start[neighbor] = neighbor_distance
                    tentative_parents[neighbor] = current
                    nodes_to_visit.add(neighbor)
        return self._construct_path(tentative_parents, target)
        
    def _construct_path(self, tentative_parents, target):
        """
        Builds a list of graph vertices as a result path of Dijkstra's 
        algorithm
        """
        if target not in tentative_parents:
            return None
        cursor = target
        path = []
        while cursor:
            path.append(cursor)
            cursor = tentative_parents.get(cursor)
        return list(reversed(path))        
