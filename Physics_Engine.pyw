from math import sqrt
import time
import pygame
import os

"""
--------------- Physics engine -----------------
Author: Mounir LBATH
Version: 1.0
Creation Date: 12/2021
-----------------------------------------------------------
"""

############################### MODEL ####################################

# Math
class Vect2:
    '''
    Vect2 class - vectors with 2 coords:
    ---------
    x : (int) x coord
    y : (int) y coord
    ---------
    '''
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # print vector
    def __str__(self):
        return "({},{})".format(self.x, self.y)
    
    # scalar multiplication
    def __mul__(self, k):
        return Vect2(k*self.x, k*self.y)
    
    # scalar division
    def __truediv__(self, k):
        return Vect2(self.x/k, self.y/k)

    # adding 2 vectors
    def __add__(self, other):
        return Vect2(self.x+other.x, self.y+other.y)
    
    #substracting 2 vectors
    def __sub__(self, other):
        return Vect2(self.x-other.x, self.y-other.y)
    
    #oppoite of vector
    def __neg__(self):
        return Vect2(-self.x, -self.y)

    # returns dot product with other
    def dot(self,other):
        return self.x*other.x+self.y*other.y

    # returns norm
    def norm(self):
        return sqrt(self.x*self.x + self.y*self.y)

    # returns norm squared
    def normSq(self):
        return self.x*self.x + self.y*self.y

    # Normalization
    def normalize(self):
        if self.norm() != 0:
            return self / self.norm()
        return self

    # converts vect2 to tuple
    def tuple(self):
        return (self.x, self.y)

    # orthogonal vector
    def orth(self):
        return Vect2(-self.y, self.x)

# Physics
class PhysicsStatic:
    '''
    PhysicsStatic class - every object in the scene which is static:
    ---------
    scene : (Scene) scene of creation
    pos : (Vect2) x and y coordinates of position
    ---------
    '''
    def __init__(self, pos = Vect2(0,0)):
        self.scene = None
        self.pos = pos
        self.id = 0

class PhysicsMoving:
    '''
    PhysicsMoving class - every object in the scene which is subject to move:
    ---------
    scene : (Scene) scene of creation
    mass : (int) mass in kg
    pos : (Vect2) x and y coordinates of position
    velocity : (vect2) x and y coord of velocity
    ---------
    '''
    def __init__(self, mass, pos = Vect2(0,0), velocity = Vect2(0,0)):
        self.scene = None
        self.mass = mass
        self.id = 0
        self.velocity = velocity
        self.pos = pos
        self.accel = Vect2(0,0)
        self.sumForce = Vect2(0,0)
        self.hasImpact = {}

    # apply forces
    def applyForce(self, f):
        self.sumForce += f

    # Using Newton's 2nd law of motion, calculate pos, accel and velocity
    def update(self):
        # set speed to 0 if too small
        if self.velocity.normSq()<0.001:
            self.velocity = Vect2(0,0)

        # called for each other object
        def handleCollision(obj):
            try:
                self.hasImpact[str(obj.id)]
            except:
                self.hasImpact[str(obj.id)] = False
            normal = self.checkCollision(obj)
            if normal!=None:
                self.inCollision = True

                # calculate normal and tangent unit vectors on the impact position
                unitNormal = normal.normalize()
                unitTang = unitNormal.orth()

                # if object got inside the other, put back
                self.pos += unitNormal*(self.radius-normal.norm())
                
                # if speed is high enough, bounce
                if not(self.hasImpact[str(obj.id)]) and self.velocity.normSq() > 0.01:
                        self.velocity = -unitNormal*2*self.velocity.dot(unitNormal)+self.velocity
                
                # Apply Restitution Coeff
                if self.hasImpact[str(obj.id)] == False:
                    self.velocity = self.velocity*0.7
                    self.hasImpact[str(obj.id)] = True

                # Normal reaction force
                if unitNormal.normSq() != 0:                    
                    normalForce = self.sumForce.dot(unitNormal)
                    #if pointing torwards inside the surface, create opposite normal reaction
                    if normalForce<0:
                        self.applyForce(-unitNormal*normalForce)
                    
            else:
                self.hasImpact[str(obj.id)] = False

        i = 0
        # for each object in the scene, handle the collision
        for obj in self.scene.staticObj:
            for i in range(2):
                handleCollision(obj)
        for obj in self.scene.movingObj:
            if obj != self:
                handleCollision(obj)

        self.accel = self.sumForce/self.mass    # Newton's second law of motion
        self.velocity = self.Integral(self.velocity, self.accel)    # integrate to get velocity
        self.pos = self.Integral(self.pos, self.velocity)   #integrate to get position
        #print(self.sumForce, self.accel, self.velocity)
        
    # integral from time
    def Integral(self, f, fDerivative):
        return f + fDerivative * self.scene.deltaTime


# Shapes
class Rect:
    '''
    Circle shape:
    ---------
    size : (Vect2) width and height
    ---------
    '''
    def __init__(self, size):
        self.size = size

class Circle:
    '''
    Circle shape:
    ---------
    radius : (int) circle radius in m
    ---------
    '''
    def __init__(self, radius):
        self.radius = radius

    # check if is colliding and react (empty, to be specified in each child class)
    def checkCollision(self, obj):
        if isinstance(obj, Circle):
            if (obj.pos-self.pos).norm() < self.radius + obj.radius:
                return self.pos-(obj.pos+(self.pos-obj.pos).normalize()*obj.radius)
            else:
                return None
        elif isinstance(obj, Rect):
            test = Vect2(self.pos.x, self.pos.y)
            if self.pos.x < obj.pos.x:
                test.x = obj.pos.x
            elif self.pos.x > obj.pos.x+obj.size.x:
                test.x = obj.pos.x+obj.size.x
            if self.pos.y > obj.pos.y:
                test.y = obj.pos.y
            elif self.pos.y < obj.pos.y-obj.size.y:
                test.y = obj.pos.y-obj.size.y
            if (self.pos-test).norm() <= self.radius:
                return (self.pos-test)
        return None



# Objects
class Ball(PhysicsMoving, Circle):
    def __init__(self, radius, mass, pos = Vect2(0,0), velocity = Vect2(0,0)):
        PhysicsMoving.__init__(self, mass, pos, velocity)
        Circle.__init__(self, radius)

class Wall(PhysicsStatic, Rect):
    def __init__(self, size, pos=Vect2(0,0)):
        PhysicsStatic.__init__(self,pos)
        Rect.__init__(self, size)

class RoundWall(PhysicsStatic, Circle):
    def __init__(self, radius, pos=Vect2(0,0)):
        PhysicsStatic.__init__(self,pos)
        Circle.__init__(self, radius)

##########################################################################

################################# VIEW ###################################

class View:
    '''
    View class - draws the scene:
    ---------
    scene : (Scene) scene of creation
    screenSize : (Vect2) width and height in pxls
    pxlSize : (int) size (in meters) corresponding to 1 pxl in the scene
    ---------
    '''
    def __init__(self):
        self.screenSize = Vect2(600,600)
        self.screenWidthInMeter = 5
        self.pxlSize = self.screenWidthInMeter/self.screenSize.x
        self.scene = None

    def setCursor(self):
        cursor_text = (
        'X                       ',
        'XX                      ',
        'X.X                     ',
        'X..X                    ',
        'X...X                   ',
        'X....X                  ',
        'X.....X                 ',
        'X......X                ',
        'X.......X               ',
        'X........X              ',
        'X.........X             ',
        'X..........X            ',
        'X......XXXXX            ',
        'X...X..X                ',
        'X..X X..X               ',
        'X.X  X..X               ',
        'XX    X..X              ',
        '      X..X              ',
        '       XX               ',
        '                        ',
        '                        ',
        '                        ',
        '                        ',
        '                        ')
        cs, mask = pygame.cursors.compile(cursor_text)
        cursor = ((24, 24), (0, 0), cs, mask)
        pygame.mouse.set_cursor(*cursor)

    def start(self):
        os.environ['SDL_VIDEO_CENTERED'] = '1'  # centers the window
        self.screen = pygame.display.set_mode(self.screenSize.tuple())
        pygame.display.set_caption('hi')
        self.setCursor()
        pygame.init()
    
    def draw(self, obj):
        if isinstance(obj, Circle):
            pygame.draw.circle(self.screen, (255,255,255), self.toPxlCoord(obj.pos).tuple(), int(obj.radius/self.pxlSize))
        elif isinstance(obj, Rect):
            pygame.draw.rect(self.screen, (255,255,255), pygame.Rect(*self.toPxlCoord(obj.pos).tuple(),int(obj.size.x/self.pxlSize),int(obj.size.y/self.pxlSize)))

    def display(self):
        self.screen.fill((0,0,0))
        for obj in self.scene.movingObj:
            self.draw(obj)
        for obj in self.scene.staticObj:
            self.draw(obj)
        pygame.display.update()

    # change meter coord to pxl coord and vice versa ((0m,0m) is bottom left corner)
    def toPxlCoord(self, coord):
        return Vect2(int(coord.x/self.pxlSize), int(self.screenSize.y-coord.y/self.pxlSize))

    def toRealCoord(self, pxlCoord):
        return Vect2(pxlCoord.x*self.pxlSize, self.pxlSize*(self.screenSize.y-pxlCoord.y))


##########################################################################

############################### CONTROLLER ###############################

class Scene:
    '''
    Scene class - the scene which contains the objects:
    ---------
    ---------
    '''
    def __init__(self):
        self.deltaTime = 0
        self.time = 0
        self.movingObj = []
        self.staticObj = []
        self.isPlaying = True # True if play, False elsewise
        self.timeOutFrame = 0
        self.view = View()
        self.view.scene = self
        self.objectId = 0 # counter of objects ids
        #self.ballCounter = 0

    # Adds objects to the scene
    def addMovingObj(self,obj):
        self.movingObj.append(obj)
        obj.id = self.objectId
        self.objectId += 1
        obj.scene = self
        return obj
    
    def addStaticObj(self,obj):
        self.staticObj.append(obj)
        obj.id = self.objectId
        self.objectId += 1
        obj.scene = self
        return obj
     

    # Run the simulation
    def run(self):
        self.view.start()
        self.timeOutFrame = time.time()
        while self.isPlaying:

            self.deltaTime = time.time() - self.timeOutFrame
            self.time += self.deltaTime
            self.timeOutFrame = time.time()
            if self.deltaTime!=0:
                print(1/self.deltaTime)

            # apply forces
            a=Vect2(*pygame.mouse.get_pos())
            for obj in self.movingObj:
                # reset sum force
                obj.sumForce = Vect2(0,0)
                
                obj.applyForce(Vect2(0,-9.81*obj.mass)) #gravity
                #obj.applyForce(-obj.velocity*10) # air/ground friction when top view

                if pygame.mouse.get_pressed()[0]:
                    # on mouse click throw the ball in the direction mouse to ball
                    obj.applyForce((obj.pos-(self.view.toRealCoord(a)))*400)
                obj.update()

            self.view.display()
            # Handle events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.isPlaying = False
                    pygame.quit()


##########################################################################



def main():
    a = Scene()
    x = a.addMovingObj(Ball(0.2,10, pos = Vect2(1,3)))
    x.velocity = Vect2(3,0)

    a.addStaticObj(Wall(Vect2(5,0.5), Vect2(0,0.5)))
    a.addStaticObj(Wall(Vect2(0.5,5), Vect2(4.75,5)))
    a.addStaticObj(Wall(Vect2(0.5,5), Vect2(-0.25,5)))
    a.addStaticObj(Wall(Vect2(5,0.5), Vect2(0,5.25)))
    a.addStaticObj(Wall(Vect2(1,0.5), Vect2(1,1)))
    a.addStaticObj(RoundWall(1, Vect2(3,1.5)))
    a.run()

############################### MAIN ###############################
main()