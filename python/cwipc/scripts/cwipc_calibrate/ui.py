import sys
from typing import List
import open3d
import open3d.visualization

from .pointcloud import Pointcloud

DRAW_OWN_AXES=True   # Use open3d coordinate drawing when false.

class UI:
    def __init__(self):
        self.winpos = 100
        
    def show_prompt(self, msg : str, isedit=False):
        stars = '*'*(len(msg)+2)
        print(stars)
        print('* ' + msg)
        print()
        print('- Inspect the pointcloud (use drag and mousehweel)')
        print('- Use +/= or -/_ to change point size')
        print('- press q or ESC when done')
        if isedit:
            print('- Select points with shift-leftclick, Deselect points with shift-rightclick')
            print('- Shift +/= or Shift -/_ to change selection indicator size')
            print('- ignore selection indicator colors, only the order is important')
        sys.stdout.flush()
        
    def show_question(self, msg : str, canretry=False) -> bool:
        answered = False
        ok = False
        while not answered:
            print('* ', msg)
            if canretry:
                print('* Press y if it is fine, n to retry, or q to abort')
            else:
                print('* Press y if it is fine, or q to abort')
            print('? ', end='')
            sys.stdout.flush()
            answer = sys.stdin.readline().strip().lower()
            if answer == 'q':
                sys.exit(1)
            ok = answer == 'y'
            answered = ok
            if canretry and answer == 'n':
                answered = True
        return ok

    def show_error(self, msg : str) -> None:
        print(msg)
        
    def show_message(self, msg : str) -> None:
        print(msg)
    
    def pick_points(self, title : str, pc : Pointcloud, from000 : bool=False) -> List[int]:
        vis = open3d.visualization.VisualizerWithEditing() # type: ignore
        vis.create_window(window_name=title, width=960, height=540, left=self.winpos, top=self.winpos)
        #self.winpos += 50
        vis.add_geometry(pc.get_o3d())
        if from000:
            viewControl = vis.get_view_control()
            viewControl.set_up([0, -1, 0])
            viewControl.set_front([0, 0, -1])
            viewControl.set_lookat([0, 0, 0])
        vis.run() # user picks points
        vis.destroy_window()
        return vis.get_picked_points()

    def show_points(self, title : str, pc : Pointcloud, from000=False) -> None:
        vis = open3d.visualization.Visualizer() # type: ignore
        vis.create_window(window_name=title, width=960, height=540, left=self.winpos, top=self.winpos)
        #self.winpos += 50
        vis.add_geometry(pc.get_o3d())
        if DRAW_OWN_AXES:
            # Draw 1 meter axes (x=red, y=green, z=blue)
            axes = open3d.geometry.LineSet()
            axes.points = open3d.utility.Vector3dVector([[0,0,0], [1,0,0], [0,1,0], [0,0,1]])
            axes.lines = open3d.utility.Vector2iVector([[0,1], [0,2], [0,3]])
            axes.colors = open3d.utility.Vector3dVector([[1,0,0], [0,1,0], [0,0,1]])
        else:
            axes = open3d.geometry.TriangleMesh.create_coordinate_frame()
        vis.add_geometry(axes)
        if from000:
            viewControl = vis.get_view_control()
            viewControl.set_up([0, -1, 0])
            viewControl.set_front([0, 0, -1])
            viewControl.set_lookat([0, 0, 0])
        vis.run()
        vis.destroy_window()
        
