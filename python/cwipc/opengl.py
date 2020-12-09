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
        self.widget = glWidget(self)
        self.button = QtWidgets.QPushButton('Test', self)
        mainLayout = QtWidgets.QHBoxLayout()
        mainLayout.addWidget(self.widget)
        mainLayout.addWidget(self.button)
        self.setLayout(mainLayout)

class glWidget(QtOpenGL.QGLWidget):

    def __init__(self, parent):
        self.pc = None
        QtOpenGL.QGLWidget.__init__(self, parent)
        self.setMinimumSize(640, 480)

    def feed(self, pc, clear):
        if self.pc: self.pc.free()
        self.pc = pc
        self.update()
        return True
        
    def paintGL(self):
        if not self.pc: return
        print('xxxjack paint called')
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(-2.5, 0.5, -6.0)
        glColor3f( 1.0, 1.5, 0.0 )
#        glPolygonMode(GL_FRONT, GL_FILL)
#        glBegin(GL_TRIANGLES)
        util.cwipc_draw(self.pc)
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


class cwipc_opengl_window:
    def __init__(self, name):
        self.app = QtWidgets.QApplication([name])
        self.window = MainWindow()
        self.window.show()
        
    def feed(self, pc, clear):
        return self.window.widget.feed(pc, clear)
        
    def caption(self, caption):
        pass
        
    def interact(self, prompt, response, millis):
        self.app.processEvents()
        return '\0'
        
