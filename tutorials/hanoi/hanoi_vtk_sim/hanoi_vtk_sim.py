import vtk
import time
from functools import partial
import sys
sys.path.append('../../../src/smsl/')
import smsl
from smsl_state import smslState
import networkx as nx

class smslTutorialHanoiScene:
    """
    Tutorial of a Hanoi game scene
    """

    def __init__(self, init_state='aaa'):
        """
        Constructor
        """
        scale_f = 0.02
        self.dim = {
            'pole_dist' : 2.0 * scale_f,
            'pole_rad' : 0.1 * scale_f,
            'pole_h' : 4.0 * scale_f,
            'cam_pos' : 20.0 * scale_f,
            'disk_size' : 0.6 * scale_f,
            'disk_thick' : 0.5 * scale_f,
            'action_step' : 0.1 * scale_f
        }
        self.pole_name = {'a': 0, 'b': 1, 'c': 2}
        self.pole_pile = [0, 0, 0]
        self.init_state = init_state
        self.pole_pos = [-self.dim['pole_dist'],0.0,self.dim['pole_dist']]
        self.disk_name = {}
        self.buffer_move_actor = []
        self.buffer_move_disk = []
        self.timer_id = []
        self.observer_id = []
        self.init_renderer()
        self.init_misc()
        self.init_camera()
        self.init_scene_objects()
        # self.example_move()
    
    def go(self, ops):
        """
        Start the demo
        - ops: [(1, 'c'), (2, 'b'), ...]
        """
        for i in ops:
            self.Op_nx(i[0], i[1])
        fnc = self.buffer_move_disk.pop(0)
        fnc()

    def example_move(self):
        """
        An example move from state_aaa to state_ccc
        """
        self.Op_nx(1, 'c')
        self.Op_nx(2, 'b')
        self.Op_nx(1, 'b')
        self.Op_nx(3, 'c')
        self.Op_nx(1, 'a')
        self.Op_nx(2, 'c')
        self.Op_nx(1, 'c')

        fnc = self.buffer_move_disk.pop(0)
        fnc()

    def Op_nx(self, n, x):
        """
        Format of Op_nx
        """
        self.buffer_move_disk.append(partial(self.move_disk, self.disk_name[str(n)], self.pole_name[str(x)]))
        
    def init_renderer(self):
        """
        Renderer init
        """
        # Create a renderer
        self.renderer = vtk.vtkRenderer()

        # Create a render window
        self.render_window = vtk.vtkRenderWindow()
        self.render_window.SetWindowName("Hanoi")
        self.render_window.SetSize(800, 600)
        self.render_window.AddRenderer(self.renderer)

        # Create an interactor
        self.render_window_interactor = vtk.vtkRenderWindowInteractor()
        self.render_window_interactor.SetRenderWindow(self.render_window)

        # Set up the interactor style
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.render_window_interactor.SetInteractorStyle(style)
        self.render_window_interactor.Initialize()

    def init_camera(self):
        """
        Init camera
        """
        # Set up the camera
        self.camera = vtk.vtkCamera()
        self.camera.SetPosition(self.dim['cam_pos'] * 0.75, 0, self.dim['cam_pos'])
        self.camera.SetFocalPoint(0, 0, 0)
        self.camera.SetViewUp(0, 0, 1)
        self.camera.SetViewAngle(20)
        self.renderer.SetActiveCamera(self.camera)

    def init_misc(self):
        """
        Misc stuff
        """
        # # Add coordinate axes to the renderer
        # axes = vtk.vtkAxesActor()
        # axes.SetTotalLength(2, 2, 2)
        # axes.SetShaftTypeToCylinder()
        # axes.SetCylinderRadius(0.02)
        # axes.SetConeRadius(0.1)
        # axes.SetAxisLabels(1)  # Show axis labels
        # renderer.AddActor(axes)

    def init_scene_objects(self):
        """
        Init the scene
        """

        # Create three moving disks with thickness
        disk3 = self.create_disk(
            0.6*self.dim['disk_size']*1.5, self.dim['disk_size']*1.5, 
            center=(
                0, self.pole_pos[self.pole_name[self.init_state[2]]], 
                self.dim['disk_thick']*self.pole_pile[self.pole_name[self.init_state[2]]]
            ), 
            color=(1, 0.5, 0)
        )
        self.pole_pile[self.pole_name[self.init_state[2]]] += 1
        disk2 = self.create_disk(
            0.6*self.dim['disk_size']*1.3, self.dim['disk_size']*1.3, 
            center=(
                0, self.pole_pos[self.pole_name[self.init_state[1]]], 
                self.dim['disk_thick']*self.pole_pile[self.pole_name[self.init_state[1]]]
            ), 
            color=(0, 0, 1)
        )
        self.pole_pile[self.pole_name[self.init_state[1]]] += 1
        disk1 = self.create_disk(
            0.6*self.dim['disk_size'], self.dim['disk_size'], 
            center=(
                0, self.pole_pos[self.pole_name[self.init_state[0]]], 
                self.dim['disk_thick']*self.pole_pile[self.pole_name[self.init_state[0]]]
            ), 
            color=(0.5, 0, 0.5)
        )
        self.pole_pile[self.pole_name[self.init_state[0]]] += 1

        self.disk_name['1'] = disk1
        self.disk_name['2'] = disk2
        self.disk_name['3'] = disk3

        # Create three cylinders
        _ = self.create_cylinder(
            radius=self.dim['pole_rad'], 
            height=self.dim['pole_h'], 
            center=(0, self.dim['pole_dist'], -self.dim['pole_dist']), 
            color=(1, 1, 1), orientation=(90, 0, 0)
        )
        _ = self.create_cylinder(
            radius=self.dim['pole_rad'], 
            height=self.dim['pole_h'], 
            center=(0, self.dim['pole_dist'], 0), 
            color=(1, 1, 1), orientation=(90, 0, 0)
        )
        _ = self.create_cylinder(
            radius=self.dim['pole_rad'], 
            height=self.dim['pole_h'], 
            center=(0, self.dim['pole_dist'], self.dim['pole_dist']), 
            color=(1, 1, 1), orientation=(90, 0, 0)
        )

        self.gripper = self.create_obj(
            'assb.obj', 4, (-2*self.dim['pole_dist'],0,0), (0,0,230))

    def create_disk(self, 
                    inner_radius=0.5, 
                    outer_radius=1.0, 
                    center=(0,0,0), 
                    color=(1,0,0)):
        """
        Init disk
        """

        torus = vtk.vtkParametricTorus()
        torus.SetRingRadius(inner_radius)
        torus.SetCrossSectionRadius(outer_radius - inner_radius)

        torus_source = vtk.vtkParametricFunctionSource()
        torus_source.SetParametricFunction(torus)
        torus_source.Update()

        color_mapper = vtk.vtkPolyDataMapper()
        color_mapper.SetInputData(torus_source.GetOutput())
        actor = vtk.vtkActor()
        actor.SetMapper(color_mapper)
        actor.GetProperty().SetColor(color)
        actor.SetPosition(center)
        self.renderer.AddActor(actor)
        return actor
    
    def create_cylinder(self, 
                        radius=0.2, 
                        height=1.0, 
                        center=(0,0,0), 
                        color=(0,1,0),
                        orientation=(0, 0, 0)):
        """
        Init cylinder
        """

        cylinder = vtk.vtkCylinderSource()
        cylinder.SetRadius(radius)
        cylinder.SetHeight(height)
        cylinder.SetCenter(center)
        cylinder.SetResolution(100)
        cylinder.Update()

        color_mapper = vtk.vtkPolyDataMapper()
        color_mapper.SetInputData(cylinder.GetOutput())

        actor = vtk.vtkActor()
        actor.SetMapper(color_mapper)
        actor.GetProperty().SetColor(color)
        actor.SetOrientation(orientation)

        self.renderer.AddActor(actor)

        return actor

    def create_obj(self, file, scale, pos, rot):
        """
        Load obj file
        """
        obj_reader = vtk.vtkOBJReader()
        obj_reader.SetFileName(file)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(obj_reader.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.SetScale(scale, scale, scale)
        actor.SetPosition(pos)  
        actor.RotateX(rot[0])
        actor.RotateY(rot[1])
        actor.RotateZ(rot[2])
        self.renderer.AddActor(actor)
        return actor

    def move_disk(self, disk, to):
        """
        Move a disk to a pole
        - to: 0, 1, 2 
        """
        
        y_to = -self.dim['pole_dist'] + to * self.dim['pole_dist']
        _,y,z = disk.GetPosition()
        frm = (y - (-self.dim['pole_dist'])) / self.dim['pole_dist']
        z_to = self.dim['disk_thick'] * self.pole_pile[to]

        self.buffer_move_actor.append(partial(
            self.move_actor, [self.gripper], 
            (0, y, self.dim['pole_h']*1.05))
        )
        self.buffer_move_actor.append(partial(
            self.move_actor, [self.gripper], 
            (0, y, z))
        )

        self.buffer_move_actor.append(partial(
            self.move_actor, [disk, self.gripper], 
            (0, y, self.dim['pole_h']*1.05))
        )
        self.buffer_move_actor.append(partial(
            self.move_actor, [disk, self.gripper], 
            (0, y_to, self.dim['pole_h']*1.05))
        )
        self.buffer_move_actor.append(partial(
            self.move_actor, [disk, self.gripper],
            (0, y_to, z_to))
        )

        self.buffer_move_actor.append(partial(
            self.move_actor, [self.gripper], 
            (0, y_to, self.dim['pole_h']*1.05))
        )
        self.pole_pile[to] += 1
        self.pole_pile[round(frm)] -= 1
        fnc = self.buffer_move_actor.pop(0)
        fnc()
        
    def move_actor(self, actors, target_position):
        """
        Move an actor
        """
        for actor in actors:
            self.timer_id.append(self.render_window_interactor.CreateRepeatingTimer(10))
            timer_callback = smslUtlTimerCallback_MoveActor(
                actor, target_position, self.render_window, 
                self.render_window_interactor, 
                self.dim['action_step'], 
                self
            )
            self.observer_id.append(self.render_window_interactor.AddObserver(
                vtk.vtkCommand.TimerEvent, timer_callback.execute_move_actor
            ))

class smslUtlTimerCallback_MoveActor:
    """
    Helper func move actor
    """

    def __init__(self, 
                 actor, 
                 target_position, 
                 render_window,
                 interactor,
                 step,
                 scene):
        """
        Constructor
        """
        self.actor = actor
        self.target_position = target_position
        self.step = step
        self.render_window = render_window
        self.interactor = interactor
        self.scene = scene

    def execute_move_actor(self, caller, event):
        """
        Execute motion
        """
        current_position = self.actor.GetPosition()
        if current_position == self.target_position:
            self.end()

        # Calculate the direction vector to the target position
        direction = [
            target - current \
            for target, current in zip(self.target_position, current_position)
        ]
        distance = vtk.vtkMath.Norm(direction)
        normalized_direction = [d / distance for d in direction]

        # Calculate the new position
        new_position = [
            current + self.step * direction \
            for current, direction in zip(current_position, normalized_direction)
        ]

        # Set the new position for the actor
        self.actor.SetPosition(new_position)

        # Render the scene
        self.render_window.Render()
        if distance < self.step:
            self.end()
    
    def end(self):
        for timer_id in self.scene.timer_id:
            self.interactor.DestroyTimer(timer_id)
        for observer_id in self.scene.observer_id:
            self.interactor.RemoveObserver(observer_id)
        if self.scene.buffer_move_actor:
            fnc = self.scene.buffer_move_actor.pop(0)
            fnc()
        elif self.scene.buffer_move_disk:
            fnc = self.scene.buffer_move_disk.pop(0)
            fnc()
        return


def main():
    
    start   = 'aab'
    end     = 'ccc'

    sm = smsl.smslStateMachine('../../../examples/hanoi.json')
    scene = smslTutorialHanoiScene(start)
    path = sm.state_machine[0].shortest_edge_path(
        smslState('State_'+start), 
        smslState('State_'+end)
    )
    
    ops = []
    for op in path:
        _op = op[0]['OP']
        print(_op)
        ops.append((_op.split('_')[1][0], _op.split('_')[1][1]))
    
    # Start the rendering loop
    scene.go(ops)
    scene.render_window.Render()
    scene.render_window_interactor.Start()

if __name__ == "__main__":
    main()

