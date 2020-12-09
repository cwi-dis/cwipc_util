import math
from OpenGL.GL import *
from OpenGL.GLU import *
from PySide2 import QtGui
from PySide2 import QtCore
from PySide2 import QtWidgets
from PySide2 import QtOpenGL
from . import util

class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.button = QtWidgets.QPushButton('Test', self)
        self.widget = glWidget(self)
        mainLayout = QtWidgets.QHBoxLayout()
        mainLayout.addWidget(self.widget)
        mainLayout.addWidget(self.button)
        self.setLayout(mainLayout)

class glViewpointMixin:
    def __init__(self):
        self.eyeAngle = 0
        self.eyeHeight = 1.8
        self.eyeDistance = 6
        
    def mousePressEvent(self, event):
        self.origMouseButton = event.button()
        self.origMousePosition = (event.x(), event.y())
        
    def mouseReleaseEvent(self, event):
        self.mouseMoveEvent(event)
        self.origMouseButton = None
        self.origMousePosition = None
        
    def mouseMoveEvent(self, event):
        newMousePosition = (event.x(), event.y())
        dx, dy = newMousePosition[0]-self.origMousePosition[0], newMousePosition[1]-self.origMousePosition[1]
        self.origMousePosition = newMousePosition
        if self.origMouseButton == QtCore.Qt.LeftButton:
            self.eyeAngle += dx / 100
        if self.origMouseButton == QtCore.Qt.RightButton:
            self.eyeHeight += dy / 100
        self.camPositionChanged()
        
    def wheelEvent(self, event):
        self.eyeDistance += event.delta() / 100
        self.camPositionChanged()
        
    def camPositionChanged(self):
        x = self.eyeDistance * math.sin(self.eyeAngle)
        y = self.eyeHeight
        z = self.eyeDistance * math.cos(self.eyeAngle)
        self.setCamPosition((x, y, z))
        self.setCamLookat((0, y, 0))
        
class glWidget(glViewpointMixin, QtOpenGL.QGLWidget):

    def __init__(self, parent):
        self.objects = []
        glViewpointMixin.__init__(self)
        QtOpenGL.QGLWidget.__init__(self, parent)
        self.setMinimumSize(640, 480)
        self.camPosition = (1, 1.8, 6)
        self.camLookat = (0, 0, 0)
        self.camUp = (0, 1, 0)
        self.clearColor = (0.2, 0.4, 0.6, 1)
        self.perspective = (60, 1.33, 0.01, 10.0)
        self.origMouseButton = None
        self.origMousePosition = None
        
    def setCamPosition(self, position):
        self.camPosition = position
        self.update()
        
    def setCamLookat(self, lookat):
        self.camLookat = lookat
        self.update()
        
    def setCamUp(self, up):
        self.camUp = up
        self.update()
        
    def setPerspective(self, perspective):
        self.perspective = perspective
        self.updatePerspective()
        
    def updatePerspective(self):
        self.makeCurrent()
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(*self.perspective)

    def addObject(self, object):
        self.objects.append(object)
        
    def prePaintGL(self):
        self.makeCurrent()
        glClearColor(*self.clearColor)       
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(*self.camPosition, *self.camLookat, *self.camUp)
        
    def postPaintGL(self):
        glFlush()

    def paintGL(self):
        self.prePaintGL()
        for o in self.objects:
            o.paintGL()
        self.postPaintGL()

    def initializeGL(self):
        glClearDepth(1.0)
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()                    
        gluPerspective(45.0,1.33,0.1, 100.0) 
        glMatrixMode(GL_MODELVIEW)

class glObject:
    def __init__(self, parent):
        self.parent = parent
        parent.addObject(self)
        
    def update(self):
        self.parent.update()
        
    def prePaintGL(self):
        pass
        
    def postPaintGL(self):
        pass
        
    def paintGL(self):
        self.prePaintGL()
        self.postPaintGL()

class glPositionedObject(glObject):
    # xxxjack does not seem to work....

    def __init__(self, parent):
        super(glPositionedObject, self).__init__(parent)
        self.translation = None
        self.rotation = None
        self.scale = None
        
    def setTranslation(self, translation):
        self.translation = translation
        
    def setRotation(self, rotation):
        self.rotation = rotation
        
    def setScale(self, scale):
        if type(scale) != type(()):
            scale = tuple(scale)
        if len(scale) != 3:
            scale = scale * 3
        self.scale = scale
        
    def prePaintGL(self):
        glPushMatrix()
        glLoadIdentity()
        if self.scale:
            glScalef(*self.scale)
        if self.rotation:
            glRotatef(*self.rotation)
        if self.translation:
            glTranslatef(*self.translation)
        
    def postPaintGL(self):
        glPopMatrix()

def tmp_util_cwipc_draw(pc):
    """xxxjack workaround for apparently two different versions of OpenGL being used on Mac by Python code and C++ code"""
    points = pc.get_points()
    glBegin(GL_POINTS)
    for p in points:
        glColor3ub(p.r, p.g, p.b)
        glVertex3f(p.x, p.y, p.z)
    glEnd()
    
class glCwipcObject(glObject):

    def __init__(self, parent):
        super(glCwipcObject, self).__init__(parent)
        self.pc = None
        
    def feed(self, pc, clear):
        # if self.pc: self.pc.free()
        self.pc = pc
        self.update()
        return True
            
    def paintGL(self):
        if not self.pc: return
        self.prePaintGL()
        print('xxxjack paint called')
        #util.cwipc_draw(self.pc)
        tmp_util_cwipc_draw(self.pc)
        self.postPaintGL()
        
class glGridObject(glObject):

    def paintGL(self):
        self.prePaintGL()
        # draw grid
        glBegin(GL_LINES)
        glColor3f(0.5, 0.5, 0.5)
        for pos in range(-5, 5+2, 2):
            glVertex3f(-5, 0, pos)
            glVertex3f(5, 0, pos)
            glVertex3f(pos, 0, -5)
            glVertex3f(pos, 0, 5)
        glEnd()
        # Draw axes
        glBegin(GL_LINES)
        glColor3f(1, 0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(1, 0, 0)
        glColor3f(0, 1, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 1, 0)
        glColor3f(0, 0, 1)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 1)
        glEnd()
        self.postPaintGL()
        
class cwipc_opengl_window:
    def __init__(self, name):
        self.app = QtWidgets.QApplication([name])
        self.window = MainWindow()
        self.pc_object = glCwipcObject(self.window.widget)
        self.grid_object = glGridObject(self.window.widget)
        self.window.show()
        
    def feed(self, pc, clear):
        return self.pc_object.feed(pc, clear)
        
    def caption(self, caption):
        pass
        
    def run(self):
        return self.app.exec_()
        
    def stop(self):
        self.app.quit()
        
if __name__ == '__main__':
    w = cwipc_opengl_window("__main__")
    w.run()
