import sys
import os.path
from PySide2.QtUiTools import QUiLoader
from PySide2.QtWidgets import QApplication, QMainWindow
from PySide2.QtCore import QFile, QIODevice
import cwipc.opengl
from . import cwipc_gui_ui

class MainWindow(QMainWindow, cwipc_gui_ui.Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)

def load_window_from_ui():
    ui_file_name = "cwipc_gui.ui"
    my_dir = os.path.dirname(__file__)
    ui_file_name = os.path.join(my_dir, ui_file_name)
    ui_file = QFile(ui_file_name)
    if not ui_file.open(QIODevice.ReadOnly):
        print("Cannot open {}: {}".format(ui_file_name, ui_file.errorString()))
        sys.exit(-1)
    loader = QUiLoader()
    loader.registerCustomWidget(cwipc.opengl.QOpenGLWidget_cwipc)
    window = loader.load(ui_file)
    ui_file.close()
    if not window:
        print(loader.errorString())
        sys.exit(-1)
    return window
    
def load_window_from_py():
    return MainWindow()
    
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = load_window_from_py()
    pc_object = cwipc.opengl.glCwipcObject(window.openGLWidget)
    grid_object = cwipc.opengl.glGridObject(window.openGLWidget)
    window.show()

    sys.exit(app.exec_())
