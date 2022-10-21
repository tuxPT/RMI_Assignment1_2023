
import sys
from croblink import *
from math import *
import math
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, challenge, outfile):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.challenge = challenge
        self.outfile = outfile

        if self.challenge == "1":
            self.track = []

        self.ACTIONS = {'STAY', 'LEFT', 'TOP', 'RIGHT', 'BOTTOM'}
        self.state = "GO"
        self.previous_angle = 0

        if self.challenge == "2":
            self.map = [[" " for j in range(1,50)] for i in range(1,22)]
            #print(len(self.map)*len(self.map[0]))
            self.map[10][24] = "I"
            #print(self.map)


            

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        self.readSensors()
        self.angle = self.measures.compass
        self.diffpos = [24 - self.measures.x, 10 - self.measures.y]

        

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
            

    def potential_paths(self, x, y):
        xn, yn = x+2, y
        if self.map[yn][xn] not in ['-', '|', '?']:
            self.map[yn][xn] = 'U'
        xn, yn = x-2, y
        if self.map[yn][xn] not in ['-', '|', '?']:
            self.map[yn][xn] = 'U'
        xn, yn = x, y+1
        if self.map[yn][xn] not in ['-', '|', '?']:
            self.map[yn][xn] = 'U'
        xn, yn = x, y-1
        if self.map[yn][xn] not in ['-', '|', '?']:
            self.map[yn][xn] = 'U'


    def wander(self):
        """ center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.1)
        else:
            print('Go')
            self.driveMotors(0.1,0.1) """

        if self.challenge == "1":
            self.wanderC1()
        elif self.challenge == "2":
            self.wanderC2()


    def wanderC1(self):

        print(f'{"Line sensor: "}{self.measures.lineSensor}')
        
        # Rotate left
        if self.measures.lineSensor[0] == '1':
            #self.driveMotors(-0.12,0.15)
            self.driveMotors(-0.08,0.1)
            self.track.append("left")

        # Rotate right
        elif self.measures.lineSensor[6] == '1':
            #self.driveMotors(0.15,-0.12)
            self.driveMotors(0.1,-0.08)
            self.track.append("right")  

        # Adjust left
        elif self.measures.lineSensor[1] == '1':
            #self.driveMotors(0.08,0.15)
            self.driveMotors(0.08,0.1)

        # Adjust right
        elif self.measures.lineSensor[5] == '1':
            #self.driveMotors(0.15,0.08)
            self.driveMotors(0.1,0.08)

        # Forward
        elif '1' in self.measures.lineSensor[2:5]:
            #self.driveMotors(0.15, 0.15)
            self.driveMotors(0.1, 0.1)      



    def wanderC2(self):
        
        x,y = self.get_correct_measures()

        x = round(x)
        y = round(y)

       
        
        # height correction
        #y = 20 - round(y)

        print(f'{"Pos: "}{x},{y}')
        print(f'{"Compass: "}{self.measures.compass}')
        print(f'{"Line sensor: "}{self.measures.lineSensor}')

        # every odd line
        if y % 2 == 1:
            self.map[y][x] = "|"
            self.potential_paths(x, y)
        # every odd column and even line
        elif x % 2 == 1:
            self.map[y][x] = "-"
            self.potential_paths(x, y)
        else:
            pass

        if self.measures.lineSensor.count('1') == 7:
            xn, yn, has_neighbor = self.unknown_pos(x, y, 'left')
            self.save_unknown(xn, yn, has_neighbor)
            xn, yn, has_neighbor = self.unknown_pos(x, y, 'right')
            self.save_unknown(xn, yn, has_neighbor)


        if self.measures.lineSensor[0] == '1':
            if self.measures.lineSensor.count('1') > 3:
                xn, yn, has_neighbor = self.unknown_pos(x, y, 'left')
                self.save_unknown(xn, yn, has_neighbor)
                ux, uy, *_ = self.unknown_pos(x, y, 'right')
                if  self.map[uy][ux] == 'U':
                    self.map[uy][ux] = ''

            #print('Rotate left')
            #self.driveMotors(-0.12,0.15)
        elif self.measures.lineSensor[6] == '1':
            if self.measures.lineSensor.count('1') > 3:
                xn, yn, has_neighbor = self.unknown_pos(x, y, 'right')
                self.save_unknown(xn, yn, has_neighbor)
                ux, uy, *_ = self.unknown_pos(x, y, 'left')
                if  self.map[uy][ux] == 'U':
                    self.map[uy][ux] = ''
        
        else:
            ux, uy, *_ = self.unknown_pos(x, y, 'left')
            if  self.map[uy][ux] == 'U':
                self.map[uy][ux] = ''
            ux, uy, *_ = self.unknown_pos(x, y, 'right')
            if  self.map[uy][ux] == 'U':
                self.map[uy][ux] = ''
        
        if self.state == "STOP":
            self.driveMotors(0, 0)
            print('STOP')
            xn, yn, has_neighbor =self.unknown_pos(x, y, 'left')
            if self.map[yn][xn] == "?":
                self.state = "R_LEFT"
                self.angle = (self.measures.compass - 90) % 360
                if self.angle > 180:
                    self.angle -= 180
                print('angle : ' + str(self.angle))
                return
            xn, yn, has_neighbor = self.unknown_pos(x, y, 'right')
            if self.map[yn][xn] == "?":
                self.state = "R_RIGHT"
                self.angle = (self.measures.compass + 90) % 360
                if self.angle > 180:
                    self.angle -= 180
                print('angle : ' + str(self.angle))
                return
        elif self.state == "GO":
            if self.measures.lineSensor[1] == '1':
                print('GO')
                self.driveMotors(0.08,0.15)
            elif self.measures.lineSensor[5] == '1':
                
                print('GO')
                self.driveMotors(0.15,0.08)
            elif '1' in self.measures.lineSensor[2:5]:
                print('GO')
                self.driveMotors(0.15, 0.15)
            else:
                self.state = "STOP"
        elif self.state == "R_RIGHT":
            print('R_RIGHT')
            if abs(self.measures.compass - self.angle) <= 1:
                self.state = "GO"
            if abs(self.measures.compass - self.angle) > abs(self.measures.compass - self.angle):
                self.state = "R_LEFT"
            else:
                self.driveMotors(0.15, -0.12)
        elif self.state == "R_LEFT":
            print('R_LEFT')
            if abs(self.measures.compass - self.angle) <= 1:
                self.state = "GO"
            if abs(self.measures.compass - self.angle) > abs(self.measures.compass - self.angle):
                self.state = "R_RIGHT"
            else:
                self.driveMotors(-0.12, 0.15)
        elif self.state == "BACK":
            self.driveMotors(-0.15, -0.15)
        else:
            pass

        self.save_map()
        

    def find_next(self, x, y):
        unexplored = [(row.index('?'), index) for index, row in enumerate(reversed(self.map)) if '?' in row]
        
        return min(unexplored, key=lambda spot: math.dist(spot, (x, y)))
            

    def unknown_pos(self, x, y, leftright):
        calc = [0, 0]
        # up
        if 80 < self.measures.compass < 100:
            if leftright == 'left':
                calc = [-1, 0]
            else:
                calc = [1, 0]
        # right
        elif -10 < self.measures.compass < 10:
            if leftright == 'left':
                calc = [0, 1]
            else:
                calc = [0, -1]
        # left
        elif -170 < self.measures.compass < -180 or 170 < self.measures.compass < 180:
            if leftright == 'left':
                calc = [0, -1]
            else:
                calc = [0, 1]
        # down
        elif -100 < self.measures.compass < -80:
            if leftright == 'left':
                calc = [1, 0]
            else:
                calc = [-1, 0]
            
        x+= calc[0]
        y+= calc[1]

        return [x, y, calc != [0, 0]]

    # menor distância entre dois pontos
    def distance(self, head, pos):
        dx = abs(head.x - pos.x)
        dy = abs(head.y - pos.y)
        return dx + dy

    # devolve uma lifo sendo que o ultimo é na verdade a proxima posicao
    def find_path(self, cameFrom, end, start):
        current = end
        deque = [current]
        while cameFrom[current] != start:
            current = cameFrom[current]
            deque.append(current)

        return deque

    # Search A_Star com dicionários
    def search(self, start, goal, bodies):
        closedset = set() # coordenadas visitadas
        openset = [start] # coordenadas por visitar
        cameFrom = {} # dicionario/mapa, dada uma posicao retorna a posicao anterior
        Gdict = collections.defaultdict(lambda: math.inf) # dicionario/mapa, dada uma posicao retorna o custo acumulado
        Gdict[start] = 0
        t0 = time.process_time()
        t1 = time.process_time()
        while t1 - t0 < 3 and openset != []:
            current = openset[0]
            if current == goal and current != start:
                return self.find_path(cameFrom, current, start)

            openset.remove(current)
            closedset.add(current)
            neighbors = list(self.valid_actions(current, bodies, closedset).keys())
            neighbors.sort(key=lambda x: self.distance(x, goal))

            for pos in neighbors:
                gscore = Gdict[current] + self.distance(current,pos)

                if gscore < Gdict[pos]:
                    cameFrom[pos] = current
                    Gdict[pos] = gscore

            openset += [pos for pos in neighbors if pos not in openset]
            t1 = time.process_time()
        return []


    def save_unknown(self, x, y, has_neighbor):
        if self.map[y][x] not in ['-', '|', 'I'] and has_neighbor:
            if y % 2 == 1 and x % 2 == 0:
                self.map[y][x] = "?"
            elif x % 2 == 1 and y % 2 == 0:
                self.map[y][x] = "?"

    def save_map(self):
        s = ""
        for row in reversed(self.map):
            for val in row:
                s += str(val)
            s += "\n"

        with open(outfile, "w+") as f:
            f.truncate(0)
            f.write(s)
            f.close()
        
    def get_correct_measures(self):
        m = [self.measures.x + self.diffpos[0], self.measures.y + self.diffpos[1]]
        #print("measures: " + str(m))
        return m

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
challenge = 1

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--challenge" or sys.argv[i] == "-c") and i != len(sys.argv) - 1:
        challenge = sys.argv[i + 1]
    elif (sys.argv[i] == "--outfile" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        outfile = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host, challenge, outfile)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
