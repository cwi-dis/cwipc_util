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

class glWidget(QtOpenGL.QGLWidget):

    def __init__(self, parent):
        self.objects = []
        QtOpenGL.QGLWidget.__init__(self, parent)
        self.setMinimumSize(640, 480)

    def addObject(self, object):
        self.objects.append(object)
        
    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        for o in self.objects:
            o.paintGL()
        glFlush()

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
        #glLoadIdentity()
        if self.scale:
            glScalef(*self.scale)
        if self.rotation:
            glRotatef(*self.rotation)
        if self.translation:
            glTranslatef(*self.translation)
        
    def postPaintGL(self):
        glPopMatrix()
  
class glCwipcObject(glPositionedObject):

    def __init__(self, parent):
        super(glCwipcObject, self).__init__(parent)
        self.pc = None
        
    def feed(self, pc, clear):
        if self.pc: self.pc.free()
        self.pc = pc
        self.update()
        return True
            
    def paintGL(self):
        if not self.pc: return
        self.prePaintGL()
        print('xxxjack paint called')
#        glTranslatef(-2.5, 0.5, -6.0)
#        glColor3f( 1.0, 1.5, 0.0 )
#        glPolygonMode(GL_FRONT, GL_FILL)
#        glBegin(GL_TRIANGLES)
        util.cwipc_draw(self.pc)
        self.postPaintGL()
        
class cwipc_opengl_window:
    def __init__(self, name):
        self.app = QtWidgets.QApplication([name])
        self.window = MainWindow()
        self.pc_object = glCwipcObject(self.window.widget)
        self.pc_object.setTranslation((-2.5, 0.5, -6.0))
        self.window.show()
        
    def feed(self, pc, clear):
        return self.pc_object.feed(pc, clear)
        
    def caption(self, caption):
        pass
        
    def interact(self, prompt, response, millis):
        self.app.processEvents()
        return '\0'
        
